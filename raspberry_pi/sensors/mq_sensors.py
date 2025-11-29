"""MQ Gas Sensors Module using ADS1015 I2C ADC"""

import logging
import time
from typing import Dict, Any

logger = logging.getLogger('mq_sensors')

try:
    from Adafruit_ADS1x15 import ADS1015
    ADS_AVAILABLE = True
except ImportError:
    ADS_AVAILABLE = False
    logger.warning("Adafruit_ADS1x15 library not available; MQ sensors will use mock values")


class MQSensors:
    """MQ Series Gas Sensors with ADS1015 I2C ADC"""

    DEFAULT_GAIN = 1  # +/-4.096V

    def __init__(self, sensor_config: Dict[str, Any], adc_config: Dict[str, Any]):
        self.sensor_config = sensor_config
        self.sensors = {}
        self.gain = adc_config.get('gain', self.DEFAULT_GAIN)
        self.address = adc_config.get('address', 0x48)

        self.ads = None
        if ADS_AVAILABLE:
            try:
                self.ads = ADS1015(address=self.address)
                logger.info("ADS1015 initialized on I2C address 0x%02X", self.address)
            except Exception as exc:
                logger.error("Failed to initialize ADS1015: %s", exc)

        for name, info in sensor_config.items():
            if info.get('enabled', False):
                self.sensors[name] = {
                    'channel': info.get('channel', 0),
                    'calibration': info.get('calibration', 1.0),
                    'baseline': info.get('baseline', 0.0)
                }
                logger.info("MQ sensor enabled: %s on channel %s", name, info.get('channel', 0))

    def _read_adc(self, channel: int) -> int:
        if self.ads is None:
            # Return pseudo reading for development
            simulated = 1500 + (channel * 100)
            logger.debug("Simulated ADC read for channel %s: %s", channel, simulated)
            return simulated

        try:
            value = self.ads.read_adc(channel, gain=self.gain)
            logger.debug("ADC read channel %s: %s", channel, value)
            return value
        except Exception as exc:
            logger.error("Error reading ADS1015 channel %s: %s", channel, exc)
            return 0

    def read_sensor(self, name: str) -> float:
        sensor = self.sensors.get(name)
        if not sensor:
            return None

        raw = self._read_adc(sensor['channel'])
        calibrated = (raw - sensor.get('baseline', 0.0)) * sensor['calibration']
        return round(calibrated, 2)

    def read_all(self) -> Dict[str, float]:
        readings = {}
        for name in self.sensors.keys():
            try:
                readings[name] = self.read_sensor(name)
            except Exception as exc:
                logger.error("Error reading MQ sensor %s: %s", name, exc)
                readings[name] = None
        return readings

    def calibrate(self, samples: int = 50, delay: float = 0.1) -> Dict[str, float]:
        baselines = {}
        for name, sensor in self.sensors.items():
            values = []
            for _ in range(samples):
                values.append(self._read_adc(sensor['channel']))
                time.sleep(delay)
            baseline = sum(values) / len(values)
            sensor['baseline'] = baseline
            baselines[name] = baseline
            logger.info("Calibrated %s baseline: %.2f", name, baseline)
        return baselines

    def get_status(self) -> Dict[str, bool]:
        return {name: self.ads is not None for name in self.sensors.keys()}
