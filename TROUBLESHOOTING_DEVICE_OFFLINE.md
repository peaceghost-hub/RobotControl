# Device Offline Troubleshooting Guide

## Problem: Dashboard shows "Robot Device Offline"

This guide helps you diagnose why your Raspberry Pi robot controller appears offline on the dashboard.

---

## Quick Checklist

- [ ] Raspberry Pi is powered on
- [ ] Robot controller process (`main.py`) is running
- [ ] Raspberry Pi can reach the dashboard server (ping test)
- [ ] Dashboard API endpoint is accepting requests
- [ ] Config file has correct dashboard URL
- [ ] No firewall blocking port 5000

---

## Step-by-Step Debugging

### 1. Check if Robot Controller is Running

**SSH into your Raspberry Pi:**
```bash
ssh pi@<pi-ip>
```

**Check process status:**
```bash
ps aux | grep main.py
```

**Expected output:** Should show `python3 main.py` process running.

**If not running, start it:**
```bash
cd /home/pi/RobotControl
python3 raspberry_pi/main.py
```

### 2. Check Dashboard URL in Config

**Verify config.json:**
```bash
cat raspberry_pi/config.json | grep -A 2 dashboard_api
```

**Expected output:**
```json
"dashboard_api": {
  "base_url": "http://<dashboard-ip>:5000",
  "endpoints": {...}
}
```

**Fix if needed:**
- Replace `<dashboard-ip>` with actual dashboard server IP
- Ensure port is `5000` (or whatever you configured)
- Use IP address, not hostname (unless mDNS is configured)

### 3. Test Network Connectivity

**From Raspberry Pi, test connection to dashboard:**
```bash
# Test connectivity
curl -v http://<dashboard-ip>:5000/api/status

# Should return JSON with device status
```

**If connection refused:**
- Dashboard server is not running
- Wrong IP address
- Wrong port
- Firewall blocking connection

**Fix:**
```bash
# On dashboard machine, verify Flask is running
python3 dashboard/app.py

# Or if using systemd service
sudo systemctl status robotcontrol
```

### 4. Check Robot Controller Logs

**View real-time logs:**
```bash
# If running systemd service
sudo journalctl -u robotcontrol -f

# If running manually in terminal, check console output
```

**Look for errors containing:**
- "Dashboard API" 
- "Connection error"
- "Failed to send status"

### 5. Verify API Endpoints

The robot sends status to `/api/status` (POST) every 10 seconds.

**Check dashboard endpoint manually:**
```bash
# From Pi terminal
curl -X POST http://<dashboard-ip>:5000/api/status \
  -H "Content-Type: application/json" \
  -d '{
    "online": true,
    "battery": 85.0,
    "signal_strength": -75,
    "system_info": {"cpu": 45, "memory": 60},
    "device_id": "robot_01"
  }'
```

**Expected response:**
```json
{"status": "success", "message": "Status updated"}
```

**If error 404 or 500:**
- Endpoint doesn't exist
- Dashboard app crashed
- Database not initialized

### 6. Check Database (Dashboard)

**Verify database is initialized:**
```bash
# On dashboard machine
cd dashboard
python3 -c "from database import db; print('DB OK')"
```

**If error, initialize database:**
```bash
python3 -c "from database import db, app; app.app_context().push(); db.create_all(); print('DB initialized')"
```

### 7. Monitor Status Updates in Real-Time

**On Raspberry Pi:**
```bash
# Watch API calls being sent
watch -n 2 'curl -s http://localhost:5000/api/status?device_id=robot_01 | python3 -m json.tool'
```

**On Dashboard Machine:**
```bash
# Check Flask logs for incoming requests
tail -f /tmp/flask_debug.log

# Or restart dashboard with debug output
export FLASK_ENV=development
python3 dashboard/app.py
```

---

## Common Issues & Solutions

### Issue: "Connection refused" when testing API

**Cause:** Dashboard not running or listening on wrong port

**Solution:**
```bash
# Start dashboard
cd dashboard
source dashboard_env/bin/activate
python3 app.py
```

### Issue: "Connection timeout" from Pi

**Cause:** Network unreachable, firewall, wrong IP

**Solution:**
```bash
# From Pi, test basic connectivity
ping <dashboard-ip>
telnet <dashboard-ip> 5000
```

### Issue: Status API responds but dashboard still shows offline

**Cause:** Device ID mismatch or status not being queried by dashboard

**Solution:**
1. Verify device_id in `config.json` matches database query
2. Restart dashboard app
3. Clear browser cache

### Issue: "KeyError" for endpoints in logs

**Cause:** `config.json` missing endpoint definitions

**Solution:** Check `config.json` has full endpoint paths:
```json
"endpoints": {
  "sensor_data": "/api/sensor_data",
  "gps_data": "/api/gps_data",
  "status": "/api/status"
}
```

---

## Enable Debug Logging

For more detailed output, modify `config.json`:

```json
{
  "log_level": "DEBUG"
}
```

Then restart `main.py`:
```bash
# Kill current process
pkill -f "python3 main.py"

# Restart with debug
python3 raspberry_pi/main.py
```

---

## Test with AutoStart Service

If using systemd service, test it works on boot:

```bash
# Stop manual process
pkill -f "python3 main.py"

# Start service
sudo systemctl start robotcontrol

# Check it's running
sudo systemctl status robotcontrol

# Monitor logs
sudo journalctl -u robotcontrol -f
```

---

## If Still Offline

1. **Check config.json syntax** (must be valid JSON)
   ```bash
   python3 -m json.tool raspberry_pi/config.json
   ```

2. **Verify all required fields** in config.json
   ```bash
   grep -E "base_url|device_id|endpoints" raspberry_pi/config.json
   ```

3. **Check for network interface issues**
   ```bash
   # View Pi network status
   ip addr show
   ip route show
   ```

4. **Enable verbose cURL logging**
   ```bash
   # From Pi
   curl -v --trace /tmp/curl.log \
     -X POST http://<dashboard-ip>:5000/api/status \
     -H "Content-Type: application/json" \
     -d '{"online": true, "device_id": "robot_01"}'
   
   cat /tmp/curl.log
   ```

5. **Check firewall on dashboard machine**
   ```bash
   # Linux
   sudo ufw status
   sudo ufw allow 5000/tcp
   ```

---

## Quick Restart Procedure

```bash
# Restart everything cleanly

# 1. Kill running processes
pkill -9 -f "python3 main.py"
pkill -9 -f "flask"

# 2. Wait for ports to clear
sleep 2

# 3. Start dashboard first (on dashboard machine)
cd /home/user/RobotControl/dashboard
python3 app.py &

# 4. Verify dashboard is up
sleep 3
curl http://localhost:5000/api/status

# 5. Start robot controller on Pi
ssh pi@<pi-ip> "cd /home/pi/RobotControl && python3 raspberry_pi/main.py"

# 6. Check status in dashboard
curl http://localhost:5000/api/status?device_id=robot_01
```

---

## Next Steps

If device is now showing **Online** ✓
- Check if GPS, sensor, and status data are flowing
- Verify dashboard widgets update in real-time
- Test command sending (navigate, stop, etc.)

If still offline:
- Collect all logs and error messages
- Run complete connectivity test from scratch
- Consider serial or I2C communication issues if using fallback modes
