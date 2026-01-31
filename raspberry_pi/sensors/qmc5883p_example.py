"""Minimal QMC5883P reader for Raspberry Pi (mirrors the C++ example)."""

import time

try:
    from smbus2 import SMBus
except ImportError:  # pragma: no cover
    from smbus import SMBus  # type: ignore

BUS_ID = 1
QMC5883P_ADDR = 0x2C
MODE_REG = 0x0A
CONFIG_REG = 0x0B
X_LSB_REG = 0x01
STATUS_REG = 0x09


def write_reg(bus: SMBus, reg: int, value: int) -> None:
    bus.write_byte_data(QMC5883P_ADDR, reg & 0xFF, value & 0xFF)


def init_qmc5883p(bus: SMBus) -> None:
    # Continuous mode, 200 Hz ODR, OSR=8 (0xCF matches shared example)
    write_reg(bus, MODE_REG, 0xCF)
    # Set/reset on, ±8G range
    write_reg(bus, CONFIG_REG, 0x08)
    time.sleep(0.01)


def read_vector(bus: SMBus) -> tuple[int, int, int]:
    # Wait for DRDY
    for _ in range(5):
        status = bus.read_byte_data(QMC5883P_ADDR, STATUS_REG)
        if status & 0x01:
            break
        time.sleep(0.01)

    data = bus.read_i2c_block_data(QMC5883P_ADDR, X_LSB_REG, 6)
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]

    # Convert to signed 16-bit
    if x >= 0x8000:
        x -= 0x10000
    if y >= 0x8000:
        y -= 0x10000
    if z >= 0x8000:
        z -= 0x10000

    return x, y, z


def main() -> None:
    with SMBus(BUS_ID) as bus:
        init_qmc5883p(bus)
        while True:
            x, y, z = read_vector(bus)
            print(f"X: {x:6d}\tY: {y:6d}\tZ: {z:6d}")
            time.sleep(0.1)


if __name__ == "__main__":
    main()
