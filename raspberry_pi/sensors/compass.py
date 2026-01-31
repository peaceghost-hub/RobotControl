"""Compass (magnetometer) driver.

Supports HMC5883L and QMC5883L.

Note: some QMC5883L variants do not behave correctly with SMBus block reads;
they require a plain I2C combined write(register) + read(bytes) transaction.
"""

try:
    from smbus2 import SMBus, i2c_msg
except Exception:  # pragma: no cover
    from smbus import SMBus  # type: ignore
    i2c_msg = None
import os
import fcntl
import ctypes
import struct
import math
import time
import logging

logger = logging.getLogger('compass')

class Compass:
    """Magnetometer for heading.

    Supports:
    - HMC5883L (classic register map, often at 0x1E)
    - QMC5883L (common replacement, often at 0x0D, sometimes at 0x2C/0x0C)
    """
    
    def __init__(self, bus=1):
        self.bus = SMBus(bus)
        self._bus_num = bus
        self._raw_i2c_fd: int | None = None
        self.address = None
        self.declination = 0  # Magnetic declination in radians
        self.chip = None  # 'HMC5883L' | 'QMC5883L'
        self._qmc_recovered_once = False

        # Best-effort: obtain an fd for raw I2C_RDWR ioctls (needed when smbus2
        # isn't installed but we still want a combined write(reg)+read(bytes)).
        try:
            fd = getattr(self.bus, "fd", None)
            if isinstance(fd, int) and fd >= 0:
                self._raw_i2c_fd = fd
        except Exception:
            pass

        if self._raw_i2c_fd is None:
            try:
                self._raw_i2c_fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
            except Exception:
                self._raw_i2c_fd = None
        
        # Try likely addresses (some modules vary / use different breakouts)
        # Prefer the most common QMC address (0x0D) first; 0x2C is unusual and some
        # unrelated devices can ACK there.
        possible_addresses = [0x0D, 0x0C, 0x2C, 0x1E]
        
        for addr in possible_addresses:
            try:
                # Test if device responds at this address
                self.bus.read_byte_data(addr, 0x00)
                self.address = addr
                logger.info(f"Compass found at address 0x{addr:02X}")
                break
            except Exception:
                continue
        
        if self.address is None:
            raise Exception("Compass not found on I2C (tried 0x0C, 0x2C, 0x1E, 0x0D)")

        # Identify chip type (best-effort)
        self.chip = self._detect_chip_type(self.address)

        try:
            if self.chip == 'QMC5883L':
                self._qmc_init(mode=0x01)

                # Best-effort readback for debugging
                try:
                    ctrl = self._read_reg(0x09)
                    setrst = self._read_reg(0x0B)
                    ctrl2 = None
                    try:
                        ctrl2 = self._read_reg(0x0A)
                    except Exception:
                        pass
                    if ctrl2 is None:
                        logger.info("QMC5883L regs: CTRL(0x09)=0x%02X SETRST(0x0B)=0x%02X", ctrl, setrst)
                    else:
                        logger.info(
                            "QMC5883L regs: CTRL(0x09)=0x%02X CTRL2(0x0A)=0x%02X SETRST(0x0B)=0x%02X",
                            ctrl,
                            ctrl2,
                            setrst,
                        )
                except Exception:
                    pass
            else:
                # HMC5883L setup
                self._write_reg(0x00, 0x70)  # 8-average, 15Hz, normal
                self._write_reg(0x01, 0x20)  # Gain 1.3 (matches Arduino handler)
                self._write_reg(0x02, 0x00)  # Continuous mode

            logger.info(f"Compass initialized at 0x{self.address:02X} as {self.chip}")

            # Post-init validation: if we can't enter continuous mode or never produce
            # non-zero samples, treat it as unavailable (prevents a stuck 0° heading).
            if self.chip == 'QMC5883L':
                try:
                    ctrl = self._read_reg(0x09)
                except Exception:
                    ctrl = None

                if isinstance(ctrl, int) and (ctrl & 0x03) != 0x01:
                    raise Exception(
                        f"QMC at 0x{self.address:02X} refuses continuous mode: CTRL(0x09)=0x{ctrl:02X}"
                    )

                ok = False
                for _ in range(8):
                    x, y, z = self.read_raw()
                    if (x, y, z) != (0, 0, 0):
                        ok = True
                        break
                    time.sleep(0.02)
                if not ok:
                    raise Exception(
                        f"QMC at 0x{self.address:02X} produced no valid samples (constant/zero data)"
                    )
        except Exception as e:
            logger.error(f"Failed to initialize compass at 0x{self.address:02X}: {e}")
            raise

    def _qmc_init(self, mode: int = 0x1D) -> None:
        """Initialize QMC5883L.

        The Arduino firmware uses:
          0x0B = 0x01 (SET/RESET)
          0x09 = 0x1D (OSR=512, RNG=2G, ODR=100Hz, Continuous)

        Some clones are sensitive to CONTROL2 writes, so we keep init minimal.
        """
        self._write_reg(0x09, 0x00)  # standby
        time.sleep(0.01)
        self._write_reg(0x0B, 0x01)  # set/reset period
        self._write_reg(0x09, mode & 0xFF)
        time.sleep(0.05)

        # Ensure the chip actually entered continuous mode (MODE bits should be 0b01).
        # Some devices will ACK these registers but mask out the mode bits entirely.
        try:
            ctrl = self._read_reg(0x09)
        except Exception:
            ctrl = None

        if isinstance(ctrl, int) and (ctrl & 0x03) != 0x01:
            # Try to force MODE=continuous while keeping upper bits the device reports.
            candidates: list[int] = []
            candidates.append((ctrl & 0xFC) | 0x01)
            # Common known-good values used in the wild.
            candidates.extend([0x1D, 0x19, 0x11, 0x09, 0x01])

            for val in candidates:
                try:
                    self._write_reg(0x09, val & 0xFF)
                    time.sleep(0.02)
                    rb = self._read_reg(0x09)
                    if (rb & 0x03) == 0x01:
                        logger.warning(
                            "QMC init: CONTROL(0x09) needed adjustment (was 0x%02X, now 0x%02X)",
                            ctrl,
                            rb,
                        )
                        break
                except Exception:
                    continue
            else:
                logger.warning(
                    "QMC init: device at 0x%02X refuses continuous mode (CTRL(0x09)=0x%02X). It may not be a QMC5883L.",
                    self.address,
                    ctrl,
                )

    def _qmc_soft_reset(self) -> None:
        """Best-effort QMC soft reset.

        Not all variants support it; when unsupported it may NACK.
        """
        self._write_reg(0x09, 0x00)  # standby
        time.sleep(0.005)
        self._write_reg(0x0A, 0x80)  # soft reset
        time.sleep(0.01)
        try:
            self._write_reg(0x0A, 0x00)
        except Exception:
            pass

    def _write_reg(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg & 0xFF, value & 0xFF)

    def _read_reg(self, reg: int) -> int:
        return int(self.bus.read_byte_data(self.address, reg & 0xFF))

    def _read_bytes(self, reg: int, length: int) -> list[int]:
        """Read bytes starting at `reg`.

        Prefer a raw I2C combined transaction (write register pointer then read),
        because some QMC variants don't support SMBus block read semantics.
        """
        if i2c_msg is not None:
            write = i2c_msg.write(self.address, [reg & 0xFF])
            read = i2c_msg.read(self.address, length)
            self.bus.i2c_rdwr(write, read)
            return list(read)

        # Fallback 1: use Linux I2C_RDWR ioctl to perform a proper combined
        # transaction even without smbus2.
        if self._raw_i2c_fd is not None:
            try:
                return self._i2c_rdwr_ioctl(reg & 0xFF, length)
            except Exception:
                pass

        # Fallback: avoid SMBus *block* read semantics (they break on some QMC clones).
        # Read one byte at a time via SMBus byte-data reads.
        out: list[int] = []
        base = reg & 0xFF
        for i in range(length):
            out.append(int(self.bus.read_byte_data(self.address, (base + i) & 0xFF)))
        return out

    def _i2c_rdwr_ioctl(self, reg: int, length: int) -> list[int]:
        """Combined I2C write(reg)+read(length) via Linux I2C_RDWR ioctl."""

        # Constants from linux/i2c-dev.h
        I2C_RDWR = 0x0707
        I2C_M_RD = 0x0001

        class _I2CMsg(ctypes.Structure):
            _fields_ = [
                ("addr", ctypes.c_uint16),
                ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16),
                ("buf", ctypes.c_void_p),
            ]

        class _I2CRdwrIoctlData(ctypes.Structure):
            _fields_ = [("msgs", ctypes.POINTER(_I2CMsg)), ("nmsgs", ctypes.c_uint32)]

        if self.address is None:
            raise RuntimeError("Compass address not set")

        wbuf = (ctypes.c_uint8 * 1)(reg & 0xFF)
        rbuf = (ctypes.c_uint8 * length)()

        msgs = (_I2CMsg * 2)()
        msgs[0].addr = self.address
        msgs[0].flags = 0
        msgs[0].len = 1
        msgs[0].buf = ctypes.cast(wbuf, ctypes.c_void_p)

        msgs[1].addr = self.address
        msgs[1].flags = I2C_M_RD
        msgs[1].len = length
        msgs[1].buf = ctypes.cast(rbuf, ctypes.c_void_p)

        ioctl_data = _I2CRdwrIoctlData(msgs=msgs, nmsgs=2)
        fcntl.ioctl(self._raw_i2c_fd, I2C_RDWR, ioctl_data)
        return [int(b) for b in rbuf]

    def _detect_chip_type(self, addr: int) -> str:
        """Attempt to detect HMC5883L vs QMC5883L.

        HMC5883L has ID registers 0x0A..0x0C that typically read 'H','4','3'.
        Many QMC5883L breakouts do not provide those IDs and use a different register map.
        """
        try:
            ida = self.bus.read_byte_data(addr, 0x0A)
            idb = self.bus.read_byte_data(addr, 0x0B)
            idc = self.bus.read_byte_data(addr, 0x0C)
            if bytes([ida, idb, idc]) == b'H43':
                return 'HMC5883L'
        except Exception:
            pass

        # Heuristic: your address 0x2C is treated as QMC in the Arduino firmware.
        if addr in (0x2C, 0x0D, 0x0C):
            return 'QMC5883L'
        return 'HMC5883L'
    
    def read_raw(self):
        """Read raw magnetometer data"""
        try:
            if self.chip == 'QMC5883L':
                # QMC5883L status at 0x06: DRDY(0), OVL(1), DOR(2)
                # Data starts at 0x00: X_L, X_H, Y_L, Y_H, Z_L, Z_H
                # Retry a few times if data not ready.
                data = None
                for _ in range(5):
                    try:
                        status = self._read_reg(0x06)
                    except Exception:
                        status = 0

                    drdy = bool(status & 0x01)
                    ovl = bool(status & 0x02)
                    if ovl:
                        logger.warning("QMC5883L overflow (status=0x%02X)", status)

                    if drdy or status == 0:
                        data = self._read_bytes(0x00, 6)
                        break
                    time.sleep(0.01)

                if data is None:
                    return 0, 0, 0
            else:
                # HMC5883L data starts at 0x03: X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB
                data = self._read_bytes(0x03, 6)

            # If the bus returned all 0x00 or all 0xFF, treat as invalid read.
            if data == [0, 0, 0, 0, 0, 0] or data == [255, 255, 255, 255, 255, 255]:
                logger.warning(
                    "Compass returned invalid raw bytes at 0x%02X: %s",
                    self.address,
                    ' '.join(f"{b:02X}" for b in data),
                )
                return 0, 0, 0

            # Some broken reads show a constant 0x80 followed by zeros; treat as invalid.
            if data == [0x80, 0x00, 0x00, 0x00, 0x00, 0x00]:
                logger.warning(
                    "Compass returned constant pattern at 0x%02X: %s",
                    self.address,
                    ' '.join(f"{b:02X}" for b in data),
                )

                # Dump key QMC regs at WARNING level so it shows up without
                # requiring logging configuration.
                if self.chip == 'QMC5883L':
                    try:
                        status = self._read_reg(0x06)
                    except Exception:
                        status = None
                    try:
                        ctrl = self._read_reg(0x09)
                    except Exception:
                        ctrl = None
                    try:
                        ctrl2 = self._read_reg(0x0A)
                    except Exception:
                        ctrl2 = None
                    try:
                        setrst = self._read_reg(0x0B)
                    except Exception:
                        setrst = None
                    logger.warning(
                        "QMC regs @0x%02X: STATUS(0x06)=%s CTRL(0x09)=%s CTRL2(0x0A)=%s SETRST(0x0B)=%s",
                        self.address,
                        f"0x{status:02X}" if isinstance(status, int) else "?",
                        f"0x{ctrl:02X}" if isinstance(ctrl, int) else "?",
                        f"0x{ctrl2:02X}" if isinstance(ctrl2, int) else "?",
                        f"0x{setrst:02X}" if isinstance(setrst, int) else "?",
                    )

                # One-time recovery attempt: soft reset + re-init (some parts need it,
                # some parts break if we do it repeatedly).
                if self.chip == 'QMC5883L' and not self._qmc_recovered_once:
                    self._qmc_recovered_once = True
                    try:
                        self._qmc_soft_reset()
                        self._qmc_init(mode=0x1D)
                        # Try an immediate re-read; if it works we'll return it.
                        data2 = self._read_bytes(0x00, 6)
                        if data2 not in ([0, 0, 0, 0, 0, 0], [255, 255, 255, 255, 255, 255], [0x80, 0x00, 0x00, 0x00, 0x00, 0x00]):
                            data = data2
                        else:
                            return 0, 0, 0
                    except Exception:
                        return 0, 0, 0
                else:
                    return 0, 0, 0

            if self.chip == 'QMC5883L':
                x = (data[1] << 8) | data[0]
                y = (data[3] << 8) | data[2]
                z = (data[5] << 8) | data[4]
            else:
                x = (data[0] << 8) | data[1]
                z = (data[2] << 8) | data[3]
                y = (data[4] << 8) | data[5]
            
            # Convert to signed
            if x > 32767: x -= 65536
            if y > 32767: y -= 65536
            if z > 32767: z -= 65536
            
            return x, y, z
        except Exception as e:
            logger.error(f"Error reading compass: {e}")
            return 0, 0, 0
    
    def read_heading(self):
        """Read compass heading in degrees"""
        x, y, z = self.read_raw()
        
        if x == 0 and y == 0:
            return 0.0
        
        # Calculate heading
        heading = math.atan2(y, x) * 180 / math.pi
        
        # Adjust for declination
        heading += self.declination * 180 / math.pi
        
        # Normalize to 0-360
        if heading < 0:
            heading += 360
        elif heading > 360:
            heading -= 360
        
        return heading
    
    def calibrate(self):
        """Simple calibration - rotate 360 degrees"""
        logger.info("Starting compass calibration - rotate sensor 360 degrees")
        
        min_x = min_y = 9999
        max_x = max_y = -9999
        
        start_time = time.time()
        while time.time() - start_time < 30:  # 30 seconds
            x, y, z = self.read_raw()
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            time.sleep(0.1)
        
        # Calculate offset
        offset_x = (max_x + min_x) / 2
        offset_y = (max_y + min_y) / 2
        
        logger.info(f"Calibration complete. Offsets: X={offset_x}, Y={offset_y}")
        # Note: For full calibration, you'd store and apply these offsets
        return offset_x, offset_y