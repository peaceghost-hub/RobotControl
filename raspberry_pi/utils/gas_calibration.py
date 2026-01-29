"""
Gas Sensor PPM Calibration for MQ Series Sensors
Converts ADS1115 raw ADC values to gas concentrations
"""

import logging

logger = logging.getLogger('gas_calibration')


class GasCalibration:
    """Calibrate MQ sensor raw values to PPM concentrations"""
    
    # ADS1115 settings
    ADS1115_MAX_VALUE = 32767  # 15-bit single-ended
    ADS1115_VOLTAGE_RANGE = 6.144  # Default gain 2/3
    VOLTS_PER_BIT = ADS1115_VOLTAGE_RANGE / ADS1115_MAX_VALUE
    
    # Hardware configuration
    RL = 1000.0  # Load resistor in Ohms (typical for MQ breakout boards)
    V_IN = 5.0   # Supply voltage
    
    # Sensor calibration constants from datasheets
    # Format: { 'Gas': [a, b, ratio_in_clean_air] }
    # Derived from datasheet log-log plots: PPM = a * (Rs/R0)^b
    CALIB = {
        'MQ2_Smoke':    [3697.4, -3.109, 9.83],   # Smoke/LPG detection
        'MQ2_LPG':      [2000.0, -2.95,  9.83],   # LPG specific
        'MQ135_CO2':    [110.47, -2.862, 3.60],   # CO2/Air quality
        'MQ135_NH3':    [102.2,  -2.473, 3.60],   # Ammonia
        'MQ7_CO':       [99.04,  -1.518, 27.5],   # Carbon monoxide
    }
    
    @classmethod
    def raw_to_voltage(cls, raw_adc: int) -> float:
        """Convert ADS1115 raw value to voltage"""
        return raw_adc * cls.VOLTS_PER_BIT
    
    @classmethod
    def calculate_rs(cls, volts: float) -> float:
        """Calculate sensor resistance from voltage"""
        if volts <= 0:
            return float('inf')
        # Rs = ((V_in * RL) / V_out) - RL
        return ((cls.V_IN * cls.RL) / volts) - cls.RL
    
    @classmethod
    def calculate_ppm(cls, raw_adc: int, sensor_type: str) -> float:
        """
        Calculate PPM concentration from raw ADC value
        
        Args:
            raw_adc: Raw ADC value from ADS1115
            sensor_type: One of 'MQ2_Smoke', 'MQ2_LPG', 'MQ135_CO2', 'MQ135_NH3', 'MQ7_CO'
        
        Returns:
            Gas concentration in PPM (parts per million)
        """
        if raw_adc <= 0:
            return 0.0
        
        calib = cls.CALIB.get(sensor_type)
        if not calib:
            logger.error(f"Unknown sensor type: {sensor_type}")
            return 0.0
        
        a, b, ratio_clean_air = calib
        
        # Convert to voltage
        volts = cls.raw_to_voltage(raw_adc)
        
        if volts <= 0:
            return 0.0
        
        # Calculate sensor resistance
        rs = cls.calculate_rs(volts)
        
        # Estimate R0 (resistance in clean air)
        # In production, calibrate once in fresh air and hardcode R0
        r0 = rs / ratio_clean_air
        
        # Calculate PPM: PPM = a * (Rs/R0)^b
        ratio = rs / r0
        ppm = a * (ratio ** b)
        
        return round(ppm, 2)
    
    @classmethod
    def get_all_concentrations(cls, mq2_raw: int, mq135_raw: int, mq7_raw: int) -> dict:
        """
        Calculate all gas concentrations from raw sensor values
        
        Args:
            mq2_raw: MQ-2 raw ADC value
            mq135_raw: MQ-135 raw ADC value
            mq7_raw: MQ-7 raw ADC value
        
        Returns:
            Dictionary with raw values and calculated PPM for all sensors
        """
        return {
            # Raw values (for cloud upload)
            'mq2_raw': mq2_raw,
            'mq135_raw': mq135_raw,
            'mq7_raw': mq7_raw,
            
            # Calculated PPM concentrations
            'mq2_smoke_ppm': cls.calculate_ppm(mq2_raw, 'MQ2_Smoke'),
            'mq2_lpg_ppm': cls.calculate_ppm(mq2_raw, 'MQ2_LPG'),
            'mq135_co2_ppm': cls.calculate_ppm(mq135_raw, 'MQ135_CO2'),
            'mq135_nh3_ppm': cls.calculate_ppm(mq135_raw, 'MQ135_NH3'),
            'mq7_co_ppm': cls.calculate_ppm(mq7_raw, 'MQ7_CO'),
            
            # Voltage for debugging
            'mq2_volts': round(cls.raw_to_voltage(mq2_raw), 3),
            'mq135_volts': round(cls.raw_to_voltage(mq135_raw), 3),
            'mq7_volts': round(cls.raw_to_voltage(mq7_raw), 3),
        }
    
    @classmethod
    def get_sensor_status(cls, raw_adc: int) -> str:
        """Get sensor status based on raw value"""
        if raw_adc < 500:
            return "Very Low (Check wiring)"
        elif raw_adc < 2000:
            return "Clean"
        elif raw_adc < 10000:
            return "Normal"
        elif raw_adc < 20000:
            return "Elevated"
        else:
            return "High Alert"


# Convenience function for quick calculations
def calculate_gas_concentrations(mq2: int, mq135: int, mq7: int) -> dict:
    """Quick wrapper for GasCalibration.get_all_concentrations()"""
    return GasCalibration.get_all_concentrations(mq2, mq135, mq7)
