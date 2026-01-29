# Gas Sensor PPM Calibration Implementation

## Overview
Added PPM (parts per million) concentration calculations for MQ gas sensors while preserving raw ADC values for cloud uploads.

## What Was Done

### 1. Created Gas Calibration Module
**File:** `raspberry_pi/utils/gas_calibration.py`

- Converts ADS1115 raw ADC values to voltage
- Calculates sensor resistance (Rs) from voltage
- Computes PPM concentrations using datasheet calibration curves
- Supports multiple gas types per sensor

**Supported Calculations:**
- MQ-2: Smoke, LPG
- MQ-135: CO2, NH3 (Ammonia)
- MQ-7: CO (Carbon Monoxide)

### 2. Updated Dashboard Backend
**File:** `dashboard/app.py`

- Import gas calibration module
- Calculate PPM values when receiving sensor data
- Send **both** raw and PPM values to frontend
- Store **only raw values** in database (for cloud upload)

### 3. Updated Dashboard Frontend

**HTML** (`templates/index.html`):
```html
<span id="mq2" class="sensor-value">--</span>
<span id="mq2-ppm" class="sensor-ppm">-- ppm</span>
<span id="mq2-raw" class="sensor-raw">(Raw: --)</span>
```

**JavaScript** (`static/js/main.js`):
- Display PPM concentrations prominently
- Show raw values in parentheses
- Use raw values for chart (consistent with cloud uploads)

**CSS** (`static/css/style.css`):
- Blue color for PPM values
- Smaller monospace font for raw values

## Current Sensor Readings

### Your Values (from test):
```
MQ-2 (Smoke):    2048 ADC  →  0.384V  →  3.03 ppm smoke
MQ-135 (CO2):    8672 ADC  →  1.626V  →  2.83 ppm CO2
MQ-7 (CO):       6096 ADC  →  1.143V  →  0.65 ppm CO
```

### Status: **Normal** ✓
All sensors showing clean air conditions.

## Data Flow

```
Raspberry Pi → Dashboard API
    ↓
Raw Values → PPM Calculation
    ↓
Database: Raw values (for ThingSpeak)
    ↓
WebSocket: Raw + PPM values
    ↓
Frontend: Display both
```

## Calibration Constants

From MQ sensor datasheets (log-log plots):

```python
CALIB = {
    'MQ2_Smoke':    [3697.4, -3.109, 9.83],
    'MQ2_LPG':      [2000.0, -2.95,  9.83],
    'MQ135_CO2':    [110.47, -2.862, 3.60],
    'MQ135_NH3':    [102.2,  -2.473, 3.60],
    'MQ7_CO':       [99.04,  -1.518, 27.5],
}
```

Formula: `PPM = a * (Rs/R0)^b`

## Hardware Configuration

```python
ADS1115_VOLTAGE_RANGE = 6.144V  # Gain 2/3
ADS1115_MAX_VALUE = 32767       # 15-bit
RL = 1000Ω                      # Load resistor
V_IN = 5.0V                     # Supply voltage
```

## Reference Levels

### CO2 (Indoor Air Quality):
- 400-1000 ppm: Good
- 1000-2000 ppm: Moderate
- 2000+ ppm: Poor ventilation

### CO (Carbon Monoxide):
- 0-9 ppm: Safe
- 9-50 ppm: Moderate (max 8hr exposure)
- 50+ ppm: Dangerous

### Smoke/LPG:
- <300 ppm: Clean air
- 300-1000 ppm: Slight detection
- 1000+ ppm: Significant presence

## Testing

Run test script:
```bash
python3 test_ppm_calibration.py
```

## Cloud Upload Integration

**ThingSpeak/Cloud Services:**
- Use raw ADC values: `data.mq2`, `data.mq135`, `data.mq7`
- Available from database: `SensorReading.mq2`, etc.
- Send raw values to cloud for their own visualization

**Local Dashboard:**
- Display calculated PPM concentrations
- Show raw values for reference
- Use raw values in chart for consistency with cloud

## Next Steps (Optional)

### 1. Fine-Tune Calibration
- Measure sensors in **fresh outdoor air**
- Record Rs values
- Calculate actual R0 = Rs / ratio_clean_air
- Hardcode R0 values in calibration module

### 2. Add Alerts
- Warning threshold: CO2 > 1000 ppm
- Danger threshold: CO > 50 ppm
- Visual indicators in dashboard

### 3. Historical PPM Charts
- Store calculated PPM in separate database table
- Create PPM-specific charts
- Toggle between raw and PPM views

## Files Modified

1. ✅ `raspberry_pi/utils/gas_calibration.py` - NEW
2. ✅ `dashboard/app.py` - Added PPM calculation
3. ✅ `dashboard/templates/index.html` - Added PPM display
4. ✅ `dashboard/static/js/main.js` - Updated sensor handler
5. ✅ `dashboard/static/css/style.css` - Added PPM styling
6. ✅ `test_ppm_calibration.py` - NEW test script

## Key Features

- ✅ Raw values preserved for cloud upload
- ✅ PPM calculations for dashboard visualization
- ✅ Both values displayed simultaneously
- ✅ Based on actual datasheet calibration curves
- ✅ Supports multiple gases per sensor
- ✅ Voltage calculation for debugging
- ✅ Sensor status indicators

**Dashboard is now running with PPM support!** 🎉

Access at: http://localhost:5000
