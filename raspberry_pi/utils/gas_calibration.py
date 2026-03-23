"""
Gas Sensor PPM Calibration for MQ Series Sensors
Converts ADS1115 raw ADC values to gas concentrations.

R0 values calibrated from clean-air sensor data (sensors.png, 2026-03-22)
with RL = 10 kΩ, VCC = 5 V, ADS1115 gain = 1  (VREF = 4.096 V):

  MQ-2   clean-air raw ≈ 2 112  →  Vout ≈ 0.264 V  →  Rs ≈ 179.4 kΩ
          R0 = 179 388 / CLEAN_AIR_FACTOR(9.83)  ≈ 18 249 Ω
          clean-air PPM = 3014.44 × 9.83^(−2.486) ≈ 10.3 ppm smoke  ✓

  MQ-135 clean-air raw ≈ 8 784  →  Vout ≈ 1.098 V  →  Rs ≈  35.5 kΩ
          R0 =  35 536 / CLEAN_AIR_FACTOR(3.60)  ≈  9 871 Ω
          clean-air PPM = 15 644 × 3.60^(−2.862) ≈  400 ppm CO₂  ✓

  MQ-7   clean-air raw ≈ 6 656  →  Vout ≈ 0.832 V  →  Rs ≈  50.1 kΩ
          R0 =  50 094 / CLEAN_AIR_FACTOR(27.5)  ≈  1 822 Ω
          clean-air PPM =  99.04 × 27.5^(−1.518) ≈  0.65 ppm CO   ✓
"""

import logging
from math import isfinite

logger = logging.getLogger('gas_calibration')


class GasCalibration:
    """Convert MQ sensor raw ADC readings to calibrated gas concentrations."""

    # ── ADS1115 settings (gain = 1  →  ±4.096 V full-scale) ──────────────────
    ADS1115_MAX_VALUE = 32767          # 15-bit, single-ended maximum
    ADS1115_VREF      = 4.096          # V  (gain = 1)
    VOLTS_PER_BIT     = ADS1115_VREF / ADS1115_MAX_VALUE  # ≈ 0.1250 mV/bit

    # ── Hardware ───────────────────────────────────────────────────────────────
    RL   = 10_000.0  # Ω  — load resistor (10 kΩ as specified by user)
    V_IN = 5.0       # V  — sensor supply voltage

    # ── Calibrated R0 values (Ω) ──────────────────────────────────────────────
    # Derived from clean-air ADC readings in sensors.png.
    # These are FIXED constants — do NOT recompute at runtime from live readings.
    R0 = {
        'MQ2':    18_249.0,   # Ω — re-calibrated 2026-03-22 (raw=2112 in clean indoor air)
        'MQ135':   9_871.0,   # Ω — re-calibrated 2026-03-22 (raw=8784 in clean indoor air)
        'MQ7':     1_822.0,   # Ω — re-calibrated 2026-03-22 (raw=6656 in clean indoor air)
    }

    # ── Power-law curves: PPM = a × (Rs / R0)^b ───────────────────────────────
    # Format: [a, b, R0-key]
    CALIB = {
        'MQ2_Smoke':  [3014.44,  -2.486, 'MQ2'],    # Smoke (from MQ-2 datasheet sensitivity curve)
        'MQ135_CO2':  [15_644.0, -2.862, 'MQ135'],  # CO₂  — ≈ 400 ppm in clean air ✓
        'MQ135_NH3':  [102.2,    -2.473, 'MQ135'],  # NH₃  ammonia
        'MQ7_CO':     [99.04,    -1.518, 'MQ7'],    # CO   — ≈ 0.65 ppm in clean air ✓
    }

    # ── Safety thresholds (international standards) ───────────────────────────
    # MQ-2  Smoke: warn ≥   500 ppm  (perceptible smoke / early fire)
    #              danger ≥ 5 000 ppm (heavy smoke / fire condition)
    # MQ-135 CO₂:  warn ≥ 1 000 ppm  (ASHRAE 62.1 occupied-space guideline)
    #              danger ≥  5 000 ppm (OSHA 8-hour PEL)
    # MQ-7  CO:    warn ≥    35 ppm   (NIOSH 15-minute ceiling)
    #              danger ≥   200 ppm  (NIOSH immediately dangerous to life)
    THRESHOLDS = {
        'mq2_smoke_ppm':  {'warn': 500,    'danger':  5_000},
        'mq135_co2_ppm':  {'warn': 1_000,  'danger':  5_000},
        'mq7_co_ppm':     {'warn':    35,   'danger':    200},
    }

    # ── Core methods ──────────────────────────────────────────────────────────

    @classmethod
    def raw_to_voltage(cls, raw_adc: int) -> float:
        """ADS1115 raw value → voltage across the load resistor (V)."""
        return raw_adc * cls.VOLTS_PER_BIT

    @classmethod
    def calculate_rs(cls, volts: float) -> float:
        """Voltage across RL → sensor resistance Rs (Ω).

        Circuit: VCC → Rs (sensor) → ADC_pin → RL → GND
          ⟹  Rs = RL × (VCC / Vout − 1)
        """
        if volts <= 0:
            return float('inf')
        return cls.RL * (cls.V_IN / volts - 1.0)

    @classmethod
    def calculate_ppm(cls, raw_adc: int, sensor_type: str) -> float:
        """Convert raw ADS1115 reading to gas concentration in ppm.

        Uses hard-coded R0 values calibrated in clean air (sensors.png).
        Returns 0.0 on any error so callers always receive a safe numeric value.
        """
        if raw_adc <= 0:
            return 0.0

        calib = cls.CALIB.get(sensor_type)
        if not calib:
            logger.error("Unknown sensor type: %s", sensor_type)
            return 0.0

        a, b, r0_key = calib
        r0 = cls.R0.get(r0_key, 0.0)
        if r0 <= 0:
            return 0.0

        volts = cls.raw_to_voltage(raw_adc)
        if volts <= 0:
            return 0.0

        rs = cls.calculate_rs(volts)
        if rs <= 0 or not isfinite(rs):
            return 0.0

        ratio = rs / r0
        ppm   = a * (ratio ** b)
        return round(max(0.0, ppm), 2)

    @classmethod
    def get_all_concentrations(cls, mq2_raw: int, mq135_raw: int, mq7_raw: int) -> dict:
        """Compute all gas concentrations and return an enhanced data dict."""
        return {
            # Raw values — stored in DB, available for re-calibration
            'mq2_raw':   mq2_raw,
            'mq135_raw': mq135_raw,
            'mq7_raw':   mq7_raw,

            # Calibrated PPM concentrations — what the dashboard displays
            'mq2_smoke_ppm':  cls.calculate_ppm(mq2_raw,   'MQ2_Smoke'),
            'mq135_co2_ppm':  cls.calculate_ppm(mq135_raw, 'MQ135_CO2'),
            'mq135_nh3_ppm':  cls.calculate_ppm(mq135_raw, 'MQ135_NH3'),
            'mq7_co_ppm':     cls.calculate_ppm(mq7_raw,   'MQ7_CO'),

            # Voltages — handy for debugging and re-calibration
            'mq2_volts':   round(cls.raw_to_voltage(mq2_raw),   3),
            'mq135_volts': round(cls.raw_to_voltage(mq135_raw), 3),
            'mq7_volts':   round(cls.raw_to_voltage(mq7_raw),   3),
        }

    @classmethod
    def get_sensor_status(cls, raw_adc: int) -> str:
        """Human-readable sensor health label based on raw ADC magnitude."""
        if raw_adc < 500:
            return "Very Low — check wiring"
        elif raw_adc < 2000:
            return "Clean"
        elif raw_adc < 10_000:
            return "Normal"
        elif raw_adc < 20_000:
            return "Elevated"
        else:
            return "High Alert"


def calculate_gas_concentrations(mq2: int, mq135: int, mq7: int) -> dict:
    """Convenience wrapper for GasCalibration.get_all_concentrations()."""
    return GasCalibration.get_all_concentrations(mq2, mq135, mq7)
