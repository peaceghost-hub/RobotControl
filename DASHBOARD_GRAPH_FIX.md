# Dashboard Graph Fix - MQ Sensors
**Fixed: January 29, 2026**

## Issues Found and Fixed

### Issue 1: Graph Only Showing MQ-135 Line
**Problem:** Chart only displayed MQ-135, missing MQ-2 and MQ-7 lines

**Root Cause:** [dashboard/static/js/main.js](dashboard/static/js/main.js) chart configuration only had 3 datasets:
- Temperature
- Humidity  
- MQ-135 ❌ (missing MQ-2 and MQ-7)

**Fix Applied:** Added MQ-2 and MQ-7 datasets to chart:
```javascript
datasets: [
    { label: 'Temperature (°C)', ... },
    { label: 'Humidity (%)', ... },
    { label: 'MQ-2 (Smoke/LPG)', borderColor: 'rgb(251, 191, 36)', ... },  // ✅ ADDED
    { label: 'MQ-135 (CO2)', borderColor: 'rgb(16, 185, 129)', ... },
    { label: 'MQ-7 (CO)', borderColor: 'rgb(168, 85, 247)', ... }         // ✅ ADDED
]
```

Updated chart data pushing:
```javascript
chart.data.datasets[0].data.push(data.temperature || 0);
chart.data.datasets[1].data.push(data.humidity || 0);
chart.data.datasets[2].data.push(data.mq2 || 0);      // ✅ ADDED
chart.data.datasets[3].data.push(data.mq135 || 0);
chart.data.datasets[4].data.push(data.mq7 || 0);      // ✅ ADDED
```

### Issue 2: No Readings for MQ-2
**Problem:** MQ-2 sensor always showing 0 or no data

**Root Cause:** [raspberry_pi/main.py](raspberry_pi/main.py) line 331 was filtering sensor data and only including `'mq7', 'mq135'` in the None-to-0 replacement loop, **excluding 'mq2'**

**Fix Applied:** Added 'mq2' to the sensor key list:
```python
# Before:
for key in ['temperature', 'humidity', 'mq7', 'mq135']:

# After:
for key in ['temperature', 'humidity', 'mq2', 'mq7', 'mq135']:  # ✅ ADDED mq2
```

## Verification

After these fixes, the dashboard should:
1. ✅ Display **5 lines** on the sensor graph:
   - Red: Temperature
   - Blue: Humidity
   - Yellow: MQ-2 (Smoke/LPG)
   - Green: MQ-135 (Air Quality/CO2)
   - Purple: MQ-7 (Carbon Monoxide)

2. ✅ Show **MQ-2 readings** from ADS1115 channel A0

## Files Modified
1. `/home/thewizard/RobotControl/dashboard/static/js/main.js` (chart config + update function)
2. `/home/thewizard/RobotControl/raspberry_pi/main.py` (sensor data processing)

## How to Apply

### On Dashboard Machine:
```bash
# Dashboard auto-reloads static files in debug mode
# If running in production, restart:
cd ~/RobotControl/dashboard
source dashboard_env/bin/activate  # if using venv
python3 app.py
```

### On Raspberry Pi:
```bash
# Restart the robot controller to pick up main.py changes:
cd ~/RobotControl
python3 raspberry_pi/main.py
```

Then refresh your browser at `http://localhost:5000`

## Expected Result
All three MQ sensors (MQ-2, MQ-135, MQ-7) should now:
- Show readings in the sensor value boxes
- Display as colored lines on the real-time graph
- Record data in history table
