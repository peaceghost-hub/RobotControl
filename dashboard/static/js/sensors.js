/**
 * Sensors Module - Additional sensor data handling and visualization
 */

// Sensor alert thresholds
const ALERT_THRESHOLDS = {
    temperature: {
        high: 50,  // °C
        low: -10   // °C
    },
    humidity: {
        high: 90,  // %
        low: 10    // %
    },
    mq2: 400,      // Smoke/LPG
    mq135: 400,    // CO2/Air quality
    mq7: 400,      // CO
    mq3: 400,      // Alcohol
    mq4: 400,      // Methane
    mq8: 400,      // Hydrogen
    mq9: 400       // CO/Flammable gases
};

/**
 * Initialize Sensors Module
 */
function initSensors() {
    window.addLog('info', 'Sensors module initialized');
}

/**
 * Check Sensor Alerts
 */
function checkSensorAlerts(data) {
    const alerts = [];
    
    // Temperature alerts
    if (data.temperature !== undefined) {
        if (data.temperature > ALERT_THRESHOLDS.temperature.high) {
            alerts.push({
                type: 'error',
                sensor: 'Temperature',
                message: `High temperature: ${data.temperature.toFixed(1)}°C`,
                value: data.temperature,
                threshold: ALERT_THRESHOLDS.temperature.high
            });
        } else if (data.temperature < ALERT_THRESHOLDS.temperature.low) {
            alerts.push({
                type: 'warning',
                sensor: 'Temperature',
                message: `Low temperature: ${data.temperature.toFixed(1)}°C`,
                value: data.temperature,
                threshold: ALERT_THRESHOLDS.temperature.low
            });
        }
    }
    
    // Humidity alerts
    if (data.humidity !== undefined) {
        if (data.humidity > ALERT_THRESHOLDS.humidity.high) {
            alerts.push({
                type: 'warning',
                sensor: 'Humidity',
                message: `High humidity: ${data.humidity.toFixed(1)}%`,
                value: data.humidity,
                threshold: ALERT_THRESHOLDS.humidity.high
            });
        } else if (data.humidity < ALERT_THRESHOLDS.humidity.low) {
            alerts.push({
                type: 'warning',
                sensor: 'Humidity',
                message: `Low humidity: ${data.humidity.toFixed(1)}%`,
                value: data.humidity,
                threshold: ALERT_THRESHOLDS.humidity.low
            });
        }
    }
    
    // Gas sensor alerts
    const gasSensors = ['mq2', 'mq135', 'mq7', 'mq3', 'mq4', 'mq8', 'mq9'];
    const gasNames = {
        mq2: 'Smoke/LPG',
        mq135: 'Air Quality (CO2)',
        mq7: 'Carbon Monoxide',
        mq3: 'Alcohol/Benzene',
        mq4: 'Methane/CNG',
        mq8: 'Hydrogen',
        mq9: 'CO/Flammable Gases'
    };
    
    gasSensors.forEach(sensor => {
        if (data[sensor] !== undefined && data[sensor] > ALERT_THRESHOLDS[sensor]) {
            const severity = sensor === 'mq7' ? 'error' : 'warning';
            alerts.push({
                type: severity,
                sensor: gasNames[sensor],
                message: `${gasNames[sensor]} alert: ${data[sensor]}`,
                value: data[sensor],
                threshold: ALERT_THRESHOLDS[sensor]
            });
        }
    });
    
    // Display alerts
    alerts.forEach(alert => {
        window.addLog(alert.type, alert.message);
        
        // Show browser notification for critical alerts
        if (alert.type === 'error' && 'Notification' in window) {
            if (Notification.permission === 'granted') {
                new Notification('Robot Alert', {
                    body: alert.message,
                    icon: '/static/images/robot-icon.png'
                });
            }
        }
    });
    
    return alerts;
}

/**
 * Get Sensor Status Color
 */
function getSensorStatusColor(sensorName, value) {
    if (value === undefined || value === null) {
        return '#6b7280'; // Gray for no data
    }
    
    const threshold = ALERT_THRESHOLDS[sensorName];
    
    if (sensorName === 'temperature') {
        if (value > threshold.high || value < threshold.low) {
            return '#ef4444'; // Red
        } else if (value > threshold.high - 10 || value < threshold.low + 10) {
            return '#f59e0b'; // Orange
        }
        return '#10b981'; // Green
    }
    
    if (sensorName === 'humidity') {
        if (value > threshold.high || value < threshold.low) {
            return '#f59e0b'; // Orange
        }
        return '#10b981'; // Green
    }
    
    // Gas sensors
    if (typeof threshold === 'number') {
        if (value > threshold) {
            return '#ef4444'; // Red
        } else if (value > threshold * 0.7) {
            return '#f59e0b'; // Orange
        }
        return '#10b981'; // Green
    }
    
    return '#10b981'; // Default green
}

/**
 * Request Notification Permission
 */
function requestNotificationPermission() {
    if ('Notification' in window && Notification.permission === 'default') {
        Notification.requestPermission().then(permission => {
            if (permission === 'granted') {
                window.addLog('info', 'Notifications enabled');
            }
        });
    }
}

/**
 * Format Sensor Value
 */
function formatSensorValue(sensorName, value) {
    if (value === undefined || value === null) {
        return '--';
    }
    
    if (sensorName === 'temperature') {
        return `${value.toFixed(1)} °C`;
    }
    
    if (sensorName === 'humidity') {
        return `${value.toFixed(1)} %`;
    }
    
    // Gas sensors (integer values)
    return Math.round(value).toString();
}

/**
 * Get Sensor Description
 */
function getSensorDescription(sensorName) {
    const descriptions = {
        temperature: 'Ambient temperature measured by DHT sensor',
        humidity: 'Relative humidity measured by DHT sensor',
        mq2: 'Detects: Smoke, LPG, Propane, Hydrogen',
        mq135: 'Detects: CO2, NH3, NOx, Benzene (Air Quality)',
        mq7: 'Detects: Carbon Monoxide (CO)',
        mq3: 'Detects: Alcohol, Benzene, Hexane',
        mq4: 'Detects: Methane, CNG, Natural Gas',
        mq8: 'Detects: Hydrogen Gas',
        mq9: 'Detects: CO, Flammable Gases'
    };
    
    return descriptions[sensorName] || 'Environmental sensor';
}

/**
 * Export Sensor Data as JSON
 */
async function exportSensorDataJSON() {
    try {
        const response = await fetch(`${window.CONFIG.apiBaseUrl}/api/sensor_data?limit=1000`);
        const data = await response.json();
        
        if (data.status === 'success') {
            const json = JSON.stringify(data.data, null, 2);
            const blob = new Blob([json], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            
            const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
            const filename = `sensor_data_${timestamp}.json`;
            
            const a = document.createElement('a');
            a.href = url;
            a.download = filename;
            a.click();
            
            URL.revokeObjectURL(url);
            window.addLog('success', `Data exported: ${filename}`);
        }
    } catch (error) {
        console.error('Error exporting data:', error);
        window.addLog('error', 'Failed to export data');
    }
}

// Initialize sensors module when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    initSensors();
    
    // Request notification permission after a delay
    setTimeout(requestNotificationPermission, 5000);
});

// Export functions to global scope
window.checkSensorAlerts = checkSensorAlerts;
window.getSensorStatusColor = getSensorStatusColor;
window.formatSensorValue = formatSensorValue;
window.getSensorDescription = getSensorDescription;
window.exportSensorDataJSON = exportSensorDataJSON;
