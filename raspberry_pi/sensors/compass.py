"""
Compass driver supporting QMC5883L and HMC5883L.
Includes corrected declination handling, improved QMC detection,
and fault-tolerant I²C access.
"""

import os
import fcntl
import ctypes
import math
import time
import logging

try:
    from smbus2 import SMBus, i2c_msg
except Exception:
    from smbus import SMBus
    i2c_msg = None


logger = logging.getLogger("compass")


def deg_to_rad(deg: float) -> float:
    """Convert degrees to radians."""
    return deg * math.pi / 180.0


class Compass:
    def __init__(self, bus=1, declination_deg=-0.5):
        self.bus = SMBus(bus)
        self._bus_num = bus
        self.address = None
        self.chip = None
        self._qmc_variant = None  # "L" for 5883L, "P" for 5883P
        self._qmc_recovered_once = False

        # declination stored in radians
        self.declination = deg_to_rad(declination_deg)

        # Try raw fd for ioctl fallback
        try:
            fd = getattr(self.bus, "fd", None)
            if isinstance(fd, int) and fd >= 0:
                self._raw_i2c_fd = fd
            else:
                raise Exception("no fd")
        except Exception:
            try:
                self._raw_i2c_fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
            except Exception:
                self._raw_i2c_fd = None

        # common magnetometer addresses
        addresses = [0x0D, 0x0C, 0x2C, 0x1E]
        for addr in addresses:
            try:
                self.bus.read_byte_data(addr, 0x00)
                self.address = addr
                break
            except Exception:
                continue

        if self.address is None:
            raise Exception("Compass not found on I2C bus.")

        self.chip = self._detect_chip_type(self.address)

        if self.chip in ("QMC5883L", "QMC5883P"):
            self._qmc_init()
        else:
            # classic HMC configuration
            self._write_reg(0x00, 0x70)  # averaging, rate, normal
            self._write_reg(0x01, 0x20)  # gain
            self._write_reg(0x02, 0x00)  # continuous

    # --------------------------------------------------------------
    # Detection
    # --------------------------------------------------------------
    def _detect_chip_type(self, addr):
        try:
            ida = self.bus.read_byte_data(addr, 0x0A)
            idb = self.bus.read_byte_data(addr, 0x0B)
            idc = self.bus.read_byte_data(addr, 0x0C)
            if bytes([ida, idb, idc]) == b"H43":
                return "HMC5883L"
        except Exception:
            pass

        if addr in (0x0D, 0x0C):
            self._qmc_variant = "L"
            return "QMC5883L"

        if addr in (0x2C, 0x2A):
            try:
                chip_id = self.bus.read_byte_data(addr, 0x00)
            except Exception:
                chip_id = None

            if chip_id == 0x80 or addr == 0x2A:
                self._qmc_variant = "P"
                return "QMC5883P"

            self._qmc_variant = "L"
            return "QMC5883L"
        return "HMC5883L"

    # --------------------------------------------------------------
    # Register read/write
    # --------------------------------------------------------------
    def _write_reg(self, reg, val):
        self.bus.write_byte_data(self.address, reg & 0xFF, val & 0xFF)

    def _read_reg(self, reg):
        return int(self.bus.read_byte_data(self.address, reg & 0xFF))

    def _read_bytes(self, reg, length):
        if i2c_msg is not None:
            w = i2c_msg.write(self.address, [reg & 0xFF])
            r = i2c_msg.read(self.address, length)
            self.bus.i2c_rdwr(w, r)
            return list(r)

        if self._raw_i2c_fd is not None:
            try:
                return self._i2c_rdwr_ioctl(reg, length)
            except Exception:
                pass

        # fallback: one-byte smbus reads
        return [self._read_reg((reg + i) & 0xFF) for i in range(length)]

    def _i2c_rdwr_ioctl(self, reg, length):
        I2C_RDWR = 0x0707
        I2C_M_RD = 0x0001

        class _I2CMsg(ctypes.Structure):
            _fields_ = [
                ("addr", ctypes.c_uint16),
                ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16),
                ("buf", ctypes.c_void_p),
            ]

        class _I2CRdwr(ctypes.Structure):
            _fields_ = [("msgs", ctypes.POINTER(_I2CMsg)), ("nmsgs", ctypes.c_uint32)]

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

        data = _I2CRdwr(msgs=msgs, nmsgs=2)
        fcntl.ioctl(self._raw_i2c_fd, I2C_RDWR, data)
        return [int(b) for b in rbuf]

    # --------------------------------------------------------------
    # QMC init and soft reset
    # --------------------------------------------------------------
    def _qmc_init(self):
        if self.chip == "QMC5883P" or self._qmc_variant == "P":
            # QMC5883P register map per QST datasheet
            self._write_reg(0x0A, 0x00)  # suspend
            time.sleep(0.005)
            try:
                self._write_reg(0x29, 0x06)  # axis sign config (recommended)
            except Exception:
                pass
            self._write_reg(0x0B, 0x08)  # set/reset on, 8G range
            time.sleep(0.001)
            self._write_reg(0x0A, 0xC3)  # continuous mode, 200Hz ODR, OSR defaults
            time.sleep(0.01)
            return

        # Legacy QMC5883L/clone sequence
        self._write_reg(0x09, 0x00)
        time.sleep(0.01)
        self._write_reg(0x0B, 0x01)
        self._write_reg(0x09, 0x1D)
        time.sleep(0.05)

    # --------------------------------------------------------------
    # RAW READ
    # --------------------------------------------------------------
    def read_raw(self):
        try:
            if self.chip in ("QMC5883L", "QMC5883P"):
                if self._qmc_variant == "P" or self.chip == "QMC5883P":
                    status = self._read_reg(0x09)
                    if status & 0x01 == 0:
                        time.sleep(0.01)
                    data = self._read_bytes(0x01, 6)
                else:
                    status = self._read_reg(0x06)
                    if status & 0x01 == 0:  # not ready
                        time.sleep(0.01)
                    data = self._read_bytes(0x00, 6)

                x = (data[1] << 8) | data[0]
                y = (data[3] << 8) | data[2]
                z = (data[5] << 8) | data[4]
            else:
                data = self._read_bytes(0x03, 6)
                x = (data[0] << 8) | data[1]
                z = (data[2] << 8) | data[3]
                y = (data[4] << 8) | data[5]

            # signed conversion
            if x > 32767:
                x -= 65536
            if y > 32767:
                y -= 65536
            if z > 32767:
                z -= 65536

            return x, y, z

        except Exception as e:
            logger.error(f"Compass read error: {e}")
            return 0, 0, 0

    # --------------------------------------------------------------
    # HEADING
    # --------------------------------------------------------------
    def read_heading(self):
        if not hasattr(self, '_lock'):
            import threading
            self._lock = threading.Lock()
        with self._lock:
            x, y, _ = self.read_raw()

        if x == 0 and y == 0:
            return 0.0

        # Heading formula: atan2(Y, X)
        # Y-axis is physically inverted (points LEFT instead of RIGHT),
        # so negate Y to correct the heading direction.
        heading = math.atan2(-y, x)

        # Apply declination (already in radians)
        heading += self.declination

        # Normalize 0–2π
        if heading < 0:
            heading += 2 * math.pi
        elif heading >= 2 * math.pi:
            heading -= 2 * math.pi

        return heading * 180 / math.pi

    # --------------------------------------------------------------
    # CALIBRATION
    # --------------------------------------------------------------
    def calibrate(self):
        logger.info("Rotate compass 360° for 30 seconds")

        min_x = 99999
        max_x = -99999
        min_y = 99999
        max_y = -99999

        t0 = time.time()
        while time.time() - t0 < 30:
            x, y, _ = self.read_raw()
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            time.sleep(0.1)

        off_x = (max_x + min_x) / 2
        off_y = (max_y + min_y) / 2

        logger.info(f"Calibration offsets: X={off_x}, Y={off_y}")
        return off_x, off_y