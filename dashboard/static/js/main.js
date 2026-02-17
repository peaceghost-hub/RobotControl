/**
 * Main Dashboard JavaScript
 * Handles WebSocket connections, data updates, and UI interactions
 */

// Configuration
const DASHBOARD_CONFIG = (() => {
    const script = document.getElementById('dashboard-config');
    if (!script) return {};
    try {
        return JSON.parse(script.textContent);
    } catch (error) {
        console.warn('Failed to parse dashboard config', error);
        return {};
    }
})();

const CONFIG = {
    socketNamespace: '/realtime',
    updateInterval: 1000, // ms
    chartDataPoints: 50,
    // How long we can go without receiving a status_update before marking the device offline.
    // Must be > the Pi status interval; default keeps the UI stable.
    deviceOfflineTimeoutSec: Number(DASHBOARD_CONFIG.deviceOfflineTimeoutSec || 30),
    apiBaseUrl: DASHBOARD_CONFIG.apiBaseUrl || window.location.origin,
    deviceId: DASHBOARD_CONFIG.deviceId || 'robot_01',
    supportedDevices: (DASHBOARD_CONFIG.supportedDevices || ['robot_01']).filter(Boolean),
    map: DASHBOARD_CONFIG.map || {},
    camera: DASHBOARD_CONFIG.camera || {},
    commands: DASHBOARD_CONFIG.commands || {},
    backup: DASHBOARD_CONFIG.backup || {},
    manualDefaults: {
        speed: 180,
        minSpeed: 80,
        maxSpeed: 255
    }
};

window.DASHBOARD_CONFIG = DASHBOARD_CONFIG;
window.CONFIG = CONFIG;

// Global state
const state = {
    connected: false,
    lastStatusUpdate: null,
    statusCheckInterval: null,
    connectionLostLogged: false,
    latestData: {
        sensors: {},
        gps: {},
        status: {},
        camera: null
    },
    charts: {},
    currentPage: 1,
    totalPages: 1,
    deviceId: CONFIG.deviceId,
    manualSpeed: CONFIG.manualDefaults.speed,
    autoSpeed: 120,
    manualOverride: false,
    navigationActive: false,
    controlMode: 'AUTO',
    backup: {
        enabled: Boolean(CONFIG.backup && CONFIG.backup.enabled),
        supportsCommands: Boolean(CONFIG.backup && CONFIG.backup.supportsCommands),
        active: false,
        lastUpdate: null,
        deviceId: (CONFIG.backup && CONFIG.backup.deviceId) || null,
        lastLocation: null,
        lastCommand: null,
        ttl: CONFIG.backup && CONFIG.backup.ttl ? Number(CONFIG.backup.ttl) : null
    },
    backupSpeed: 160
};

// Socket.IO connection
let socket;

/**
 * Initialize Dashboard
 */
function initDashboard() {
    console.log('Initializing Dashboard...');
    addLog('info', 'Dashboard initializing...');
    
    // Initialize status check interval (check every 2 seconds)
    state.statusCheckInterval = setInterval(checkRobotStatus, 2000);
    
    initDeviceSelector();
    initOverlaySystem();
    initZoomControls();

    // Connect to WebSocket
    connectWebSocket();
    
    // Initialize charts
    initCharts();
    
    // Set up event listeners
    setupEventListeners();
    
    // Load initial data
    loadInitialData();
    
    // Initialize sensor control
    initSensorControl();
    
    updateControlIndicators();
    updateBackupIndicators();

    const speedSlider = document.getElementById('manual-speed-slider');
    if (speedSlider) {
        speedSlider.value = state.manualSpeed;
    }
    updateElement('manual-speed-display', state.manualSpeed);

    const autoSpeedSlider = document.getElementById('auto-speed-slider');
    if (autoSpeedSlider) {
        autoSpeedSlider.value = state.autoSpeed;
    }
    updateElement('auto-speed-display', state.autoSpeed);

    const backupSpeedSlider = document.getElementById('backup-speed-slider');
    if (backupSpeedSlider) {
        backupSpeedSlider.value = state.backupSpeed;
    }
    updateElement('backup-speed-display', state.backupSpeed);

    addLog('success', 'Dashboard initialized successfully');
}

function initDeviceSelector() {
    const selector = document.getElementById('device-select');
    if (!selector) return;

    selector.innerHTML = '';
    const devices = CONFIG.supportedDevices.length ? CONFIG.supportedDevices : [CONFIG.deviceId];

    devices.forEach(id => {
        const cleanId = id.trim();
        if (!cleanId) return;
        const option = document.createElement('option');
        option.value = cleanId;
        option.textContent = cleanId;
        selector.appendChild(option);
    });

    selector.value = state.deviceId;

    selector.addEventListener('change', event => {
        state.deviceId = event.target.value;
        CONFIG.deviceId = state.deviceId;
        state.manualOverride = false;
        updateControlIndicators();
        addLog('info', `Switched to device ${state.deviceId}`);
        loadInitialData();
        loadWaypoints();
        sendRobotCommand('PING');
    });
}

/**
 * WebSocket Connection
 */
function connectWebSocket() {
    socket = io(CONFIG.socketNamespace);
    
    socket.on('connect', () => {
        console.log('WebSocket connected');
        state.connected = true;
        updateConnectionStatus('websocket', true);
        addLog('success', 'Connected to server');
        
        // Request initial data
        socket.emit('request_update', {});
    });
    
    socket.on('disconnect', () => {
        console.log('WebSocket disconnected');
        state.connected = false;
        updateConnectionStatus('websocket', false);
        // Don't force the device indicator offline here; it's driven by status_update heartbeat.
        addLog('warning', 'Disconnected from server');
    });
    
    socket.on('sensor_update', (data) => {
        handleSensorUpdate(data);
    });
    
    socket.on('gps_update', (data) => {
        handleGPSUpdate(data);
    });
    
    socket.on('status_update', (data) => {
        handleStatusUpdate(data);
    });
    
    socket.on('camera_frame', (data) => {
        handleCameraFrame(data);
    });
    
    socket.on('waypoint_update', (data) => {
        handleWaypointUpdate(data);
        addLog('info', `Waypoint ${data.action}: ${JSON.stringify(data)}`);
    });
    
    socket.on('full_update', (data) => {
        console.log('Received full update', data);
        if (data.sensors) handleSensorUpdate(data.sensors);
        if (data.gps) handleGPSUpdate(data.gps);
        if (data.status) handleStatusUpdate(data.status);
        if (data.camera) handleCameraFrame(data.camera);
        if (data.backup) handleBackupUpdate(data.backup);
        if (data.nav_status && data.nav_status.nav) updateNavStatusUI(data.nav_status.nav);
    });
    
    socket.on('connection_response', (data) => {
        console.log('Connection response:', data);
    });

    socket.on('command_update', (data) => {
        if (data && data.command) {
            addLog('info', `Command ${data.event}: ${data.command.command_type}`);
        }
    });

    socket.on('backup_update', (data) => {
        handleBackupUpdate(data);
    });

    // Robot events (e.g., obstacle detected)
    socket.on('robot_event', (event) => {
        handleRobotEvent(event);
    });
    
    // Sensor data control events
    socket.on('sensor_control', (data) => {
        handleSensorControl(data);
    });
    
    // Waypoint completion events
    socket.on('waypoint_completed', (data) => {
        handleWaypointCompletion(data);
    });
}

/**
 * Update Connection Status Indicator
 * @param {string} type - 'websocket' or 'device'
 * @param {boolean} connected - Connection status
 */
function updateConnectionStatus(type, connected) {
    const indicator = document.getElementById(`${type}-indicator`);
    const text = document.getElementById(`${type}-text`);
    
    if (!indicator || !text) return;
    
    if (connected) {
        indicator.className = 'status-dot online';
        text.textContent = type === 'websocket' ? 'Connected' : 'Online';
        // Set class for styling
        text.className = type === 'websocket' ? 'connected' : 'online';
        // Remove any inline color styles
        text.style.color = '';
    } else {
        indicator.className = 'status-dot offline';
        text.textContent = type === 'websocket' ? 'Disconnected' : 'Offline';
        // Set class for styling
        text.className = type === 'websocket' ? 'disconnected' : 'offline';
        // Remove any inline color styles
        text.style.color = '';
    }
}

/**
 * Handle Sensor Data Update
 */
function handleSensorUpdate(data) {
    state.latestData.sensors = data;
    
    // Update sensor displays
    updateElement('temperature', data.temperature ? `${data.temperature.toFixed(1)} °C` : '--');
    updateElement('humidity', data.humidity ? `${data.humidity.toFixed(1)} %` : '--');
    
    // Update MQ sensors with raw and PPM values
    // MQ-2 (Smoke/LPG)
    updateElement('mq2', data.mq2_raw || data.mq2 || '--');
    updateElement('mq2-ppm', data.mq2_smoke_ppm ? `${data.mq2_smoke_ppm.toFixed(1)} ppm` : '-- ppm');
    updateElement('mq2-raw', data.mq2_raw ? `(Raw: ${data.mq2_raw})` : '(Raw: --)');
    
    // MQ-135 (CO2)
    updateElement('mq135', data.mq135_raw || data.mq135 || '--');
    updateElement('mq135-ppm', data.mq135_co2_ppm ? `${data.mq135_co2_ppm.toFixed(1)} ppm` : '-- ppm');
    updateElement('mq135-raw', data.mq135_raw ? `(Raw: ${data.mq135_raw})` : '(Raw: --)');
    
    // MQ-7 (CO)
    updateElement('mq7', data.mq7_raw || data.mq7 || '--');
    updateElement('mq7-ppm', data.mq7_co_ppm ? `${data.mq7_co_ppm.toFixed(1)} ppm` : '-- ppm');
    updateElement('mq7-raw', data.mq7_raw ? `(Raw: ${data.mq7_raw})` : '(Raw: --)');
    
    // Update chart (use raw values for consistency)
    const chartData = {
        temperature: data.temperature,
        humidity: data.humidity,
        mq2: data.mq2_raw || data.mq2,
        mq135: data.mq135_raw || data.mq135,
        mq7: data.mq7_raw || data.mq7
    };
    updateSensorChart(chartData);
    
    // Check for alerts
    checkSensorAlerts(data);

    // Allow compass heading from sensor payloads to update the UI even without GPS fixes.
    if (data.heading !== undefined && data.heading !== null) {
        applyHeadingUpdate(data.heading, 'primary');
    }
}

/**
 * Handle GPS Update
 */
function handleGPSUpdate(data) {
    if (!data) return;

    const source = data.source || 'primary';
    const normalized = normalizeLocation(data);
    const payload = normalized ? { ...data, ...normalized } : data;
    state.latestData.gps = payload;
    if (source === 'backup') {
        state.backup.active = true;
        state.backup.lastUpdate = payload.timestamp || new Date().toISOString();
        state.backup.lastLocation = normalizeLocation(payload) || payload;
        if (payload.device_id) {
            state.backup.deviceId = payload.device_id;
        }
    } else if (state.backup.active && source === 'primary') {
        state.backup.active = false;
    }
    updateBackupIndicators();
    updateControlIndicators();
    
    const lat = payload.latitude !== undefined ? Number(payload.latitude) : NaN;
    const lon = payload.longitude !== undefined ? Number(payload.longitude) : NaN;
    const hasValidFix = Number.isFinite(lat) && Number.isFinite(lon) && lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;

    // Update GPS displays
    updateElement('gps-lat', hasValidFix ? lat.toFixed(6) : '--');
    updateElement('gps-lon', hasValidFix ? lon.toFixed(6) : '--');
    if (!applyHeadingUpdate(payload.heading, source)) {
        updateElement('gps-heading', '--');
    }
    updateElement('gps-speed', payload.speed !== undefined ? `${Number(payload.speed).toFixed(2)} m/s` : '--');
    updateElement('gps-satellites', payload.satellites !== undefined ? `${payload.satellites}` : '--');
    updateElement('gps-source', source === 'backup' ? 'Backup (LoRa)' : 'Primary (Pi)');

    // Update map only when coordinates are valid
    if (hasValidFix && window.updateRobotPosition) {
        window.updateRobotPosition(lat, lon, payload.heading, source);
    }
}

/**
 * Handle Status Update
 */
function handleStatusUpdate(data) {
    // Track when we received this status update
    state.lastStatusUpdate = new Date();
    state.connectionLostLogged = false;
    state.latestData.status = data;
    
    // Update device connection status
    updateConnectionStatus('device', data.online);
    
    if (typeof data.manual_override === 'boolean') {
        state.manualOverride = data.manual_override;
    }
    if (typeof data.navigation_active === 'boolean') {
        state.navigationActive = data.navigation_active;
    }
    if (typeof data.mode === 'string') {
        state.controlMode = data.mode.toUpperCase();
    }
    if (typeof data.fallback_active === 'boolean') {
        state.backup.active = data.fallback_active;
    }
    updateBackupIndicators();
    
    // Update online status
    const onlineElement = document.getElementById('robot-online');
    const statusDot = document.getElementById('robot-status-dot');
    
    if (data.online) {
        onlineElement.textContent = 'Online';
        onlineElement.style.color = 'var(--success-color)';
        statusDot.className = 'status-dot online';
    } else {
        onlineElement.textContent = 'Offline';
        onlineElement.style.color = 'var(--danger-color)';
        statusDot.className = 'status-dot offline';
    }
    
    // Update battery
    const batteryLevel = data.battery || 0;
    updateElement('battery-level', `${batteryLevel.toFixed(0)}%`);
    
    const batteryFill = document.getElementById('battery-fill');
    batteryFill.style.width = `${batteryLevel}%`;
    batteryFill.className = batteryLevel < 20 ? 'battery-fill low' : 'battery-fill';
    
    // Update signal
    updateElement('signal-strength', data.signal_strength ? `${data.signal_strength} dBm` : '--');
    
    // Update last update time
    if (data.last_update) {
        const lastUpdate = new Date(data.last_update);
        updateElement('last-update', lastUpdate.toLocaleTimeString());
    }
    
    // Check battery alert
    if (batteryLevel < 20 && batteryLevel > 0) {
        addLog('warning', `Low battery: ${batteryLevel.toFixed(0)}%`);
    }

    updateControlIndicators();
}

/**
 * Handle Camera Frame
 */
function handleCameraFrame(data) {
    if (window.updateCameraFeed) {
        window.updateCameraFeed(data.frame, data.timestamp);
    }
}

/**
 * Handle Waypoint Update
 */
function handleWaypointUpdate(data) {
    loadWaypoints();
}

/**
 * Handle Sensor Control Events
 */
function handleSensorControl(data) {
    const pauseBtn = document.getElementById('sensor-pause-btn');
    const startBtn = document.getElementById('sensor-start-btn');
    const stopBtn = document.getElementById('sensor-stop-btn');
    
    if (data.action === 'pause') {
        pauseBtn.disabled = true;
        startBtn.disabled = false;
        stopBtn.disabled = false;
        addLog('info', 'Sensor data collection paused');
    } else if (data.action === 'start') {
        pauseBtn.disabled = false;
        startBtn.disabled = true;
        stopBtn.disabled = false;
        addLog('info', 'Sensor data collection started');
    } else if (data.action === 'stop') {
        pauseBtn.disabled = false;
        startBtn.disabled = true;
        stopBtn.disabled = true;
        addLog('warning', 'Sensor data collection stopped');
    }
    
    // Update button states
    updateSensorControlButtons(data.enabled);
}

/**
 * Handle Waypoint Completion Events
 */
function handleWaypointCompletion(data) {
    const notification = document.getElementById('waypoint-notification');
    const message = document.getElementById('waypoint-message');
    
    message.textContent = data.message;
    notification.classList.remove('hidden');
    
    // Auto-hide after 10 seconds
    setTimeout(() => {
        notification.classList.add('hidden');
    }, 10000);
    
    addLog('success', data.message);
    
    // Reload waypoints to update the map
    loadWaypoints();
}

/**
 * Control Sensor Data Collection
 */
function controlSensorData(action) {
    fetch(`${CONFIG.apiBaseUrl}/api/sensor_data/control`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ action: action })
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'success') {
            addLog('info', data.message);
        } else {
            addLog('error', `Failed to ${action} sensor data: ${data.message}`);
        }
    })
    .catch(error => {
        addLog('error', `Error controlling sensor data: ${error.message}`);
    });
}

/**
 * Update Sensor Control Button States
 */
function updateSensorControlButtons(enabled) {
    const pauseBtn = document.getElementById('sensor-pause-btn');
    const startBtn = document.getElementById('sensor-start-btn');
    const stopBtn = document.getElementById('sensor-stop-btn');
    
    if (enabled) {
        pauseBtn.disabled = false;
        startBtn.disabled = true;
        stopBtn.disabled = false;
    } else {
        pauseBtn.disabled = true;
        startBtn.disabled = false;
        stopBtn.disabled = false;
    }
}

/**
 * Initialize Sensor Control State
 */
function initSensorControl() {
    // Get initial sensor control status
    fetch(`${CONFIG.apiBaseUrl}/api/sensor_data/control`)
        .then(response => response.json())
        .then(data => {
            if (data.status === 'success') {
                updateSensorControlButtons(data.enabled);
            }
        })
        .catch(error => {
            console.warn('Failed to get sensor control status:', error);
        });
}

/**
 * Initialize Charts
 */
function initCharts() {
    const ctx = document.getElementById('sensorChart');
    if (!ctx) return;
    
    state.charts.sensorChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {
                    label: 'Temperature (°C)',
                    data: [],
                    borderColor: 'rgb(239, 68, 68)',
                    backgroundColor: 'rgba(239, 68, 68, 0.1)',
                    tension: 0.4
                },
                {
                    label: 'Humidity (%)',
                    data: [],
                    borderColor: 'rgb(59, 130, 246)',
                    backgroundColor: 'rgba(59, 130, 246, 0.1)',
                    tension: 0.4
                },
                {
                    label: 'MQ-2 (Smoke/LPG)',
                    data: [],
                    borderColor: 'rgb(251, 191, 36)',
                    backgroundColor: 'rgba(251, 191, 36, 0.1)',
                    tension: 0.4
                },
                {
                    label: 'MQ-135 (CO2)',
                    data: [],
                    borderColor: 'rgb(16, 185, 129)',
                    backgroundColor: 'rgba(16, 185, 129, 0.1)',
                    tension: 0.4
                },
                {
                    label: 'MQ-7 (CO)',
                    data: [],
                    borderColor: 'rgb(168, 85, 247)',
                    backgroundColor: 'rgba(168, 85, 247, 0.1)',
                    tension: 0.4
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    position: 'top'
                },
                title: {
                    display: true,
                    text: 'Real-time Sensor Data'
                }
            },
            scales: {
                y: {
                    beginAtZero: true
                }
            }
        }
    });
}

/**
 * Update Sensor Chart
 */
function updateSensorChart(data) {
    const chart = state.charts.sensorChart;
    if (!chart) return;
    
    const time = new Date().toLocaleTimeString();
    
    chart.data.labels.push(time);
    chart.data.datasets[0].data.push(data.temperature || 0);
    chart.data.datasets[1].data.push(data.humidity || 0);
    chart.data.datasets[2].data.push(data.mq2 || 0);
    chart.data.datasets[3].data.push(data.mq135 || 0);
    chart.data.datasets[4].data.push(data.mq7 || 0);
    
    // Keep only last N points
    if (chart.data.labels.length > CONFIG.chartDataPoints) {
        chart.data.labels.shift();
        chart.data.datasets.forEach(dataset => dataset.data.shift());
    }
    
    chart.update('none');
}

/**
 * Check Sensor Alerts
 */
function checkSensorAlerts(data) {
    if (data.temperature > 50) {
        addLog('error', `High temperature alert: ${data.temperature.toFixed(1)}°C`);
    }
    
    if (data.mq2 > 400) {
        addLog('warning', `High smoke/gas detected: MQ-2 = ${data.mq2}`);
    }
    
    if (data.mq7 > 400) {
        addLog('error', `Carbon monoxide alert: MQ-7 = ${data.mq7}`);
    }
    
    if (data.mq135 > 400) {
        addLog('warning', `Poor air quality: MQ-135 = ${data.mq135}`);
    }
}

/**
 * Load Initial Data
 */
async function loadInitialData() {
    try {
        // Load latest sensor data
    const sensorResponse = await fetch(`${CONFIG.apiBaseUrl}/api/sensor_data/latest?device_id=${encodeURIComponent(state.deviceId)}`);
        const sensorData = await sensorResponse.json();
        if (sensorData.status === 'success' && sensorData.data) {
            handleSensorUpdate(sensorData.data);
        }
        
        // Load latest GPS
    const gpsResponse = await fetch(`${CONFIG.apiBaseUrl}/api/gps_data/latest?device_id=${encodeURIComponent(state.deviceId)}`);
        const gpsData = await gpsResponse.json();
        if (gpsData.status === 'success' && gpsData.data) {
            handleGPSUpdate(gpsData.data);
        }
        
        // Load robot status
    const statusResponse = await fetch(`${CONFIG.apiBaseUrl}/api/status?device_id=${encodeURIComponent(state.deviceId)}`);
        const statusData = await statusResponse.json();
        if (statusData.status === 'success' && statusData.data) {
            handleStatusUpdate(statusData.data);
        }
        
        await fetchBackupStatus();

        // Load waypoints
        loadWaypoints();
        
        // Load sensor history
        loadSensorHistory();
        
    } catch (error) {
        console.error('Error loading initial data:', error);
        addLog('error', 'Failed to load initial data');
    }
}

async function fetchBackupStatus() {
    if (!CONFIG.backup) {
        return;
    }

    try {
        const response = await fetch(`${CONFIG.apiBaseUrl}/api/backup/status`);
        const data = await response.json();
        if (data.status === 'success' && data.data) {
            handleBackupUpdate(data.data);
        }
    } catch (error) {
        console.warn('Failed to fetch backup status:', error);
    }
}

/**
 * Load Waypoints
 */
async function loadWaypoints() {
    try {
        const response = await fetch(`${CONFIG.apiBaseUrl}/api/waypoints?device_id=${encodeURIComponent(state.deviceId)}`);
        const data = await response.json();
        
        if (data.status === 'success') {
            displayWaypoints(data.data);
            if (window.updateMapWaypoints) {
                window.updateMapWaypoints(data.data);
            }
        }
    } catch (error) {
        console.error('Error loading waypoints:', error);
    }
}

/**
 * Display Waypoints
 */
function displayWaypoints(waypoints) {
    const listElement = document.getElementById('waypoint-list');
    const countElement = document.getElementById('waypoint-count');
    
    if (countElement) {
        countElement.textContent = waypoints.length;
    }
    if (!listElement) {
        console.warn('Waypoint list element not found');
        return;
    }
    
    if (waypoints.length === 0) {
        listElement.innerHTML = '<p class="empty-message">No waypoints set</p>';
        return;
    }
    
    listElement.innerHTML = waypoints.map(wp => `
        <div class="waypoint-item">
            <div class="waypoint-info">
                <span class="waypoint-number">#${wp.sequence}</span>
                <span>${wp.description || 'Waypoint'}</span>
                <div class="waypoint-coords">${wp.latitude.toFixed(6)}, ${wp.longitude.toFixed(6)}</div>
            </div>
            <button class="btn btn-small" onclick="deleteWaypoint(${wp.id})">🗑️</button>
        </div>
    `).join('');
}

/**
 * Send robot command to backend queue
 */
async function sendRobotCommand(command, payload = {}) {
    if (state.backup.active) {
        addLog('warning', `Primary command ${command} blocked while backup control is active`);
        return false;
    }
    try {
        const response = await fetch(`${CONFIG.apiBaseUrl}/api/commands`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                command,
                payload,
                device_id: state.deviceId
            })
        });

        const data = await response.json();

        if (data.status === 'success') {
            addLog('info', `Command queued: ${command}`);
            return true;
        }

        addLog('error', data.message || `Failed to queue command ${command}`);
        return false;
    } catch (error) {
        console.error(`Error sending command ${command}:`, error);
        addLog('error', `Command failed: ${command}`);
        return false;
    }
}

async function sendNavigationCommand(action) {
    if (state.backup.active) {
        addLog('warning', 'Navigation commands disabled while backup control is active');
        return;
    }

    const navMap = {
        'start':  { cmd: 'NAV_START',  active: true  },
        'pause':  { cmd: 'NAV_PAUSE',  active: false },
        'resume': { cmd: 'NAV_RESUME', active: true  },
        'stop':   { cmd: 'NAV_STOP',   active: false }
    };
    const entry = navMap[action];
    if (!entry) return;

    // Use WebSocket instant channel for lowest latency
    if (socket && socket.connected) {
        socket.emit('instant_command', {
            command: entry.cmd,
            payload: {},
            device_id: state.deviceId
        });
        addLog('info', `Instant nav command: ${entry.cmd}`);
    } else {
        // Fallback to DB queue when WebSocket is down
        await sendRobotCommand(entry.cmd);
    }

    state.navigationActive = entry.active;
    updateControlIndicators();
}

async function sendManualCommand(direction) {
    if (state.backup.active) {
        addLog('warning', 'Manual override unavailable while backup control is active');
        return;
    }
    state.manualOverride = true;
    updateControlIndicators();

    const payload = {
        direction,
        speed: state.manualSpeed
    };

    // Use WebSocket instant channel for lowest latency
    if (socket && socket.connected) {
        socket.emit('instant_command', {
            command: 'MANUAL_DRIVE',
            payload,
            device_id: state.deviceId
        });
        addLog('info', `Instant command: MANUAL_DRIVE ${direction}`);
    } else {
        // Fallback to HTTP queue if WebSocket is down
        await sendRobotCommand('MANUAL_DRIVE', payload);
    }
}

async function releaseManualMode() {
    if (state.backup.active) {
        addLog('warning', 'Primary manual override already released (backup active)');
        return;
    }
    // Use WebSocket instant channel for lowest latency
    if (socket && socket.connected) {
        socket.emit('instant_command', {
            command: 'MANUAL_OVERRIDE',
            payload: { mode: 'release' },
            device_id: state.deviceId
        });
        state.manualOverride = false;
        updateControlIndicators();
    } else {
        const success = await sendRobotCommand('MANUAL_OVERRIDE', { mode: 'release' });
        if (success) {
            state.manualOverride = false;
            updateControlIndicators();
        }
    }
}

async function requestManualMode() {
    if (state.backup.active) {
        addLog('warning', 'Primary manual override unavailable while backup control is active');
        return;
    }
    // Use WebSocket instant channel for lowest latency
    if (socket && socket.connected) {
        socket.emit('instant_command', {
            command: 'MANUAL_OVERRIDE',
            payload: { mode: 'hold' },
            device_id: state.deviceId
        });
        state.manualOverride = true;
        updateControlIndicators();
    } else {
        const success = await sendRobotCommand('MANUAL_OVERRIDE', { mode: 'hold' });
        if (success) {
            state.manualOverride = true;
            updateControlIndicators();
        }
    }
}

async function sendWaypointsToRobot() {
    if (state.backup.active) {
        addLog('warning', 'Cannot push waypoints while backup control is active');
        return;
    }
    // Use WebSocket instant channel for lowest latency
    if (socket && socket.connected) {
        socket.emit('instant_command', {
            command: 'WAYPOINT_PUSH',
            payload: {},
            device_id: state.deviceId
        });
        addLog('info', 'Instant command: WAYPOINT_PUSH');
    } else {
        await sendRobotCommand('WAYPOINT_PUSH');
    }
}

function updateManualSpeed(value) {
    if (state.backup.active) {
        addLog('warning', 'Primary speed control disabled while backup control is active');
        return;
    }
    const numeric = Number(value);
    state.manualSpeed = numeric;
    updateElement('manual-speed-display', numeric);
    // Use instant channel so speed change applies immediately
    if (socket && socket.connected) {
        socket.emit('instant_command', {
            command: 'MANUAL_SPEED',
            payload: { value: numeric },
            device_id: state.deviceId
        });
    } else {
        sendRobotCommand('MANUAL_SPEED', { value: numeric });
    }
}

function updateAutoSpeed(value) {
    if (state.backup.active) {
        addLog('warning', 'Primary auto speed control disabled while backup control is active');
        return;
    }
    const numeric = Number(value);
    state.autoSpeed = numeric;
    updateElement('auto-speed-display', numeric);
    sendRobotCommand('AUTO_SPEED', { value: numeric });
}

async function sendBackupCommand(command, payload = {}) {
    if (!state.backup.enabled) {
        addLog('warning', 'Backup link disabled in configuration');
        return false;
    }
    if (!state.backup.supportsCommands) {
        addLog('warning', 'Backup command forwarding is disabled');
        return false;
    }

    try {
        const response = await fetch(`${CONFIG.apiBaseUrl}/api/backup/command`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                command,
                payload,
                device_id: state.backup.deviceId || state.deviceId
            })
        });

        const data = await response.json();

        if (data.status === 'success') {
            addLog('info', `Backup command sent: ${command}`);
            return true;
        }

        addLog('error', data.message || `Failed to send backup command ${command}`);
        return false;
    } catch (error) {
        console.error(`Error sending backup command ${command}:`, error);
        addLog('error', `Backup command failed: ${command}`);
        return false;
    }
}

async function sendBackupDrive(direction) {
    const payload = {
        direction,
        speed: state.backupSpeed
    };
    return sendBackupCommand('MANUAL_DRIVE', payload);
}

async function sendBackupOverride(mode) {
    return sendBackupCommand('MANUAL_OVERRIDE', { mode });
}

function updateBackupSpeed(value) {
    const numeric = Number(value);
    state.backupSpeed = numeric;
    updateElement('backup-speed-display', numeric);

    if (state.backup.active && state.backup.supportsCommands) {
        sendBackupCommand('MANUAL_SPEED', { value: numeric });
    }
}

function handleBackupUpdate(data) {
    if (!data) return;

    const prevActive = state.backup.active;
    if (data.enabled !== undefined) {
        state.backup.enabled = Boolean(data.enabled);
    } else {
        state.backup.enabled = state.backup.enabled || Boolean(data.active) || Boolean(data.device_id);
    }
    if (data.supportsCommands !== undefined) {
        state.backup.supportsCommands = Boolean(data.supportsCommands);
    }
    state.backup.active = Boolean(data.active);
    state.backup.lastUpdate = data.last_update || data.lastUpdate || state.backup.lastUpdate;
    if (data.device_id) {
        state.backup.deviceId = data.device_id;
    }

    const location = data.last_location || data.lastLocation;
    const normalizedLocation = normalizeLocation(location);
    if (normalizedLocation) {
        state.backup.lastLocation = normalizedLocation;
    }

    if (data.last_command !== undefined) {
        state.backup.lastCommand = data.last_command;
    } else if (data.lastCommand !== undefined) {
        state.backup.lastCommand = data.lastCommand;
    }

    updateBackupIndicators();
    updateControlIndicators();

    if (prevActive !== state.backup.active) {
        addLog(state.backup.active ? 'warning' : 'success', state.backup.active ? 'LoRa backup link engaged' : 'Primary control path restored');
    }

    if (state.backup.active && state.backup.lastLocation) {
        const enriched = {
            ...state.backup.lastLocation,
            source: 'backup'
        };
        handleGPSUpdate(enriched);
    }
}

function updateBackupIndicators() {
    const pill = document.getElementById('backup-status-pill');
    const statusText = document.getElementById('backup-status-text');
    const active = state.backup.enabled && state.backup.active;
    const controlsEnabled = active && state.backup.supportsCommands;

    if (pill) {
        pill.textContent = active ? 'ACTIVE' : (state.backup.enabled ? 'Standby' : 'Disabled');
        pill.classList.toggle('danger', active);
        pill.classList.toggle('warning', active);
    }

    if (statusText) {
        statusText.textContent = active ? 'Active' : (state.backup.enabled ? 'Standby' : 'Disabled');
    }

    const lastUpdateText = state.backup.lastUpdate ? new Date(state.backup.lastUpdate).toLocaleTimeString() : '--';
    updateElement('backup-last-update', lastUpdateText);
    updateElement('backup-last-update-short', lastUpdateText);
    updateElement('backup-device-id', state.backup.deviceId || '--');

    const lastLocation = state.backup.lastLocation || {};
    updateElement('backup-lat', lastLocation.latitude !== undefined ? Number(lastLocation.latitude).toFixed(6) : '--');
    updateElement('backup-lon', lastLocation.longitude !== undefined ? Number(lastLocation.longitude).toFixed(6) : '--');
    updateElement('backup-heading', lastLocation.heading !== undefined ? `${Number(lastLocation.heading).toFixed(1)}°` : '--');
    updateElement('backup-speed', lastLocation.speed !== undefined ? `${Number(lastLocation.speed).toFixed(2)} m/s` : '--');
    updateElement('backup-last-command', formatBackupCommand(state.backup.lastCommand));

    const backupButtons = document.querySelectorAll('.backup-grid .control-key');
    backupButtons.forEach(btn => {
        btn.disabled = !controlsEnabled;
    });

    const backupEngageBtn = document.getElementById('backup-engage-btn');
    if (backupEngageBtn) {
        backupEngageBtn.disabled = !state.backup.supportsCommands;
    }

    const backupReleaseBtn = document.getElementById('backup-release-btn');
    if (backupReleaseBtn) {
        backupReleaseBtn.disabled = !controlsEnabled;
    }

    const backupSpeedSlider = document.getElementById('backup-speed-slider');
    if (backupSpeedSlider) {
        backupSpeedSlider.disabled = !controlsEnabled;
        backupSpeedSlider.value = state.backupSpeed;
    }

    updateElement('backup-speed-display', state.backupSpeed);

    if (window.setBackupMapMode) {
        window.setBackupMapMode(active);
    }
}

function formatBackupCommand(command) {
    if (!command) return '--';
    try {
        if (typeof command === 'string') {
            return command.toUpperCase();
        }
        if (command.command) {
            const name = command.command.toString().toUpperCase();
            const payload = command.payload || command.raw || {};
            const direction = payload.direction || command.direction;
            const speedVal = payload.speed || command.speed;
            const directionLabel = direction ? direction.toString().toUpperCase() : null;
            const speedLabel = speedVal !== undefined && speedVal !== null && !Number.isNaN(Number(speedVal))
                ? `${Math.round(Number(speedVal))}`
                : null;
            if (directionLabel && speedLabel) {
                return `${name}:${directionLabel}@${speedLabel}`;
            }
            if (directionLabel) {
                return `${name}:${directionLabel}`;
            }
            if (speedLabel) {
                return `${name}:SPD${speedLabel}`;
            }
            if (payload && Object.keys(payload).length) {
                return `${name}:${JSON.stringify(payload)}`;
            }
            return name;
        }
        const direction = command.direction ? command.direction.toString().toUpperCase() : null;
        const speed = typeof command.speed === 'number' && !Number.isNaN(command.speed)
            ? `${Math.round(command.speed)}`
            : null;
        if (direction && speed) {
            return `${direction} @${speed}`;
        }
        if (direction) {
            return direction;
        }
        if (speed) {
            return `SPD ${speed}`;
        }
        if (command.raw && typeof command.raw === 'object') {
            return JSON.stringify(command.raw);
        }
        return '--';
    } catch (error) {
        console.warn('Failed to format backup command', error);
        return '--';
    }
}

/**
 * Add Waypoint
 */
async function addWaypoint(latitude, longitude, description = '') {
    try {
        const response = await fetch(`${CONFIG.apiBaseUrl}/api/waypoints`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                waypoints: [{
                    latitude: parseFloat(latitude),
                    longitude: parseFloat(longitude),
                    description: description
                }],
                device_id: state.deviceId
            })
        });
        
        const data = await response.json();
        
        if (data.status === 'success') {
            addLog('success', `Waypoint added: ${latitude}, ${longitude}`);
            loadWaypoints();
        } else {
            addLog('error', `Failed to add waypoint: ${data.message}`);
        }
    } catch (error) {
        console.error('Error adding waypoint:', error);
        addLog('error', 'Failed to add waypoint');
    }
}

/**
 * Delete Waypoint
 */
async function deleteWaypoint(waypointId) {
    try {
        const response = await fetch(`${CONFIG.apiBaseUrl}/api/waypoints/${waypointId}`, {
            method: 'DELETE'
        });
        
        const data = await response.json();
        
        if (data.status === 'success') {
            addLog('info', `Waypoint ${waypointId} deleted`);
            loadWaypoints();
        }
    } catch (error) {
        console.error('Error deleting waypoint:', error);
        addLog('error', 'Failed to delete waypoint');
    }
}

/**
 * Clear All Waypoints
 */
async function clearAllWaypoints() {
    if (!confirm('Are you sure you want to clear all waypoints?')) {
        return;
    }
    
    try {
        const response = await fetch(`${CONFIG.apiBaseUrl}/api/waypoints/clear`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ device_id: state.deviceId })
        });
        
        const data = await response.json();
        
        if (data.status === 'success') {
            addLog('info', 'All waypoints cleared');
            loadWaypoints();
        }
    } catch (error) {
        console.error('Error clearing waypoints:', error);
        addLog('error', 'Failed to clear waypoints');
    }
}

/**
 * Load Sensor History
 */
async function loadSensorHistory(page = 1) {
    try {
        const response = await fetch(`${CONFIG.apiBaseUrl}/api/sensor_data?limit=20&device_id=${encodeURIComponent(state.deviceId)}`);
        const data = await response.json();
        
        if (data.status === 'success') {
            displaySensorHistory(data.data);
        }
    } catch (error) {
        console.error('Error loading sensor history:', error);
    }
}

/**
 * Display Sensor History
 */
function displaySensorHistory(data) {
    const tbody = document.getElementById('data-table-body');
    
    if (data.length === 0) {
        tbody.innerHTML = '<tr><td colspan="6" class="empty-message">No data available</td></tr>';
        return;
    }
    
    tbody.innerHTML = data.map(row => `
        <tr>
            <td>${new Date(row.timestamp).toLocaleString()}</td>
            <td>${row.temperature ? row.temperature.toFixed(1) : '--'}</td>
            <td>${row.humidity ? row.humidity.toFixed(1) : '--'}</td>
            <td>${row.mq2 || '--'}</td>
            <td>${row.mq135 || '--'}</td>
            <td>${row.mq7 || '--'}</td>
        </tr>
    `).join('');
}

/**
 * Add Log Entry
 */
function addLog(type, message) {
    const logContainer = document.getElementById('log-container');
    const time = new Date().toLocaleTimeString();
    
    const logEntry = document.createElement('div');
    logEntry.className = `log-entry ${type}`;
    logEntry.innerHTML = `
        <span class="log-time">${time}</span>
        <span class="log-message">${message}</span>
    `;
    
    logContainer.insertBefore(logEntry, logContainer.firstChild);
    
    // Keep only last 100 logs
    while (logContainer.children.length > 100) {
        logContainer.removeChild(logContainer.lastChild);
    }
}

/**
 * Update control mode indicator and button states
 */
function updateControlIndicators() {
    const modeLabel = document.getElementById('control-mode-label');
    if (modeLabel) {
        let modeText = state.controlMode || 'AUTO';
        if (state.backup.active) {
            modeText = 'BACKUP';
        } else if (state.manualOverride) {
            modeText = 'MANUAL';
        }
        modeLabel.textContent = modeText;
        modeLabel.classList.toggle('danger', state.manualOverride || state.backup.active);
        modeLabel.classList.toggle('warning', state.backup.active);
    }

    const manualButtons = document.querySelectorAll('.manual-grid .control-key');
    manualButtons.forEach(btn => {
        const disableForBackup = state.backup.active;
        btn.disabled = disableForBackup || (!state.manualOverride && btn.id !== 'manual-stop-btn');
    });

    const releaseBtn = document.getElementById('manual-release-btn');
    if (releaseBtn) {
        releaseBtn.disabled = !state.manualOverride || state.backup.active;
    }

    const navButtonIds = ['nav-start-btn', 'nav-pause-btn', 'nav-resume-btn', 'nav-stop-btn', 'send-waypoints-btn'];
    navButtonIds.forEach(id => {
        const button = document.getElementById(id);
        if (button) {
            button.disabled = state.backup.active;
        }
    });
}

/**
 * Check if the robot is still connected by verifying the last status update time
 */
function checkRobotStatus() {
    if (!state.lastStatusUpdate || !state.latestData.status) return;

    const secondsSinceReceive = (Date.now() - state.lastStatusUpdate.getTime()) / 1000;
    const timeoutSec = Number.isFinite(CONFIG.deviceOfflineTimeoutSec) ? CONFIG.deviceOfflineTimeoutSec : 30;

    // If we haven't RECEIVED a status update recently, mark offline once.
    if (secondsSinceReceive > timeoutSec) {
        if (state.latestData.status.online) {
            handleStatusUpdate({
                ...state.latestData.status,
                online: false
            });
        } else {
            updateConnectionStatus('device', false);
        }

        if (!state.connectionLostLogged) {
            addLog('warning', `Robot connection lost - no status updates for ${Math.round(secondsSinceReceive)}s`);
            state.connectionLostLogged = true;
        }
    }
}

/**
 * Update Element Content
 */
function updateElement(id, content) {
    const element = document.getElementById(id);
    if (element) {
        if (element.tagName === 'INPUT' || element.tagName === 'SELECT') {
            element.value = content;
        } else {
            element.textContent = content;
        }
    }
}

/**
 * Update heading UI from any data source (GPS, sensors, backup link).
 */
function applyHeadingUpdate(value, source = 'primary') {
    if (value === undefined || value === null) {
        return false;
    }

    const numeric = Number(value);
    if (!Number.isFinite(numeric)) {
        return false;
    }

    updateElement('gps-heading', `${numeric.toFixed(1)}°`);

    // Keep backup heading label in sync even when only primary data is available.
    updateElement('backup-heading', `${numeric.toFixed(1)}°`);

    // Also feed the compass widget so heading shows even without NAV_STATUS events
    if (compassState.heading === null || source === 'primary') {
        compassState.heading = numeric;
        drawCompass();
    }

    if (!state.latestData.gps) {
        state.latestData.gps = {};
    }
    state.latestData.gps.heading = numeric;
    state.latestData.gps.headingSource = source;

    const lat = Number(state.latestData.gps.latitude);
    const lon = Number(state.latestData.gps.longitude);
    const hasFix = Number.isFinite(lat) && Number.isFinite(lon);
    if (hasFix && typeof window.updateRobotPosition === 'function') {
        const markerSource = state.latestData.gps.source || source;
        window.updateRobotPosition(lat, lon, numeric, markerSource);
    }
    return true;
}

/**
 * Setup Event Listeners
 */
function setupEventListeners() {
    // Toolbar overlay toggles
    const sensorsBtn = document.getElementById('open-sensors-btn');
    if (sensorsBtn) sensorsBtn.addEventListener('click', () => toggleOverlay('sensors', true));
    const dataBtn = document.getElementById('open-data-btn');
    if (dataBtn) dataBtn.addEventListener('click', () => toggleOverlay('data', true));
    const waypointsBtn = document.getElementById('open-waypoints-btn');
    if (waypointsBtn) waypointsBtn.addEventListener('click', () => toggleOverlay('waypoints', true));
    const statusBtn = document.getElementById('open-status-btn');
    if (statusBtn) statusBtn.addEventListener('click', () => toggleOverlay('status'));
    const logsBtn = document.getElementById('toggle-logs-btn');
    if (logsBtn) logsBtn.addEventListener('click', () => toggleOverlay('logs'));

    const resetLayoutBtn = document.getElementById('reset-layout-btn');
    if (resetLayoutBtn) resetLayoutBtn.addEventListener('click', () => {
        if (window.__resetOverlayLayout) {
            window.__resetOverlayLayout();
        } else {
            localStorage.removeItem('dashboardLayout.v1');
            window.location.reload();
        }
    });

    // Layout toggle: fit-screen ↔ scroll
    const toggleLayoutBtn = document.getElementById('toggle-layout-btn');
    if (toggleLayoutBtn) {
        // Restore saved preference
        const savedLayout = localStorage.getItem('dashboardLayoutMode');
        if (savedLayout === 'scroll') {
            document.body.classList.add('layout-scroll');
            toggleLayoutBtn.textContent = '📐 Scroll';
            // Leaflet needs a size recalc after switching
            setTimeout(() => { if (window.map) window.map.invalidateSize(); }, 100);
        }
        toggleLayoutBtn.addEventListener('click', () => {
            const isScroll = document.body.classList.toggle('layout-scroll');
            toggleLayoutBtn.textContent = isScroll ? '📐 Scroll' : '📐 Fit Screen';
            localStorage.setItem('dashboardLayoutMode', isScroll ? 'scroll' : 'fit');
            // Leaflet map must recalculate its container size
            setTimeout(() => {
                if (window.map) window.map.invalidateSize();
            }, 150);
        });
    }

    // Add waypoint manually
    document.getElementById('add-waypoint-manual-btn').addEventListener('click', () => {
        const lat = document.getElementById('waypoint-lat').value;
        const lon = document.getElementById('waypoint-lon').value;
        const desc = document.getElementById('waypoint-desc').value;
        
        if (lat && lon) {
            addWaypoint(lat, lon, desc);
            // Clear inputs
            document.getElementById('waypoint-lat').value = '';
            document.getElementById('waypoint-lon').value = '';
            document.getElementById('waypoint-desc').value = '';
        } else {
            alert('Please enter latitude and longitude');
        }
    });
    
    // Clear waypoints
    document.getElementById('clear-waypoints-btn').addEventListener('click', clearAllWaypoints);
    
    // Refresh data
    document.getElementById('refresh-data-btn').addEventListener('click', loadSensorHistory);
    
    // Clear logs
    document.getElementById('clear-logs-btn').addEventListener('click', () => {
        document.getElementById('log-container').innerHTML = '';
        addLog('info', 'Logs cleared');
    });
    
    // Export data
    document.getElementById('export-data-btn').addEventListener('click', exportSensorData);
    
    // Sensor control buttons
    document.getElementById('sensor-pause-btn').addEventListener('click', () => controlSensorData('pause'));
    document.getElementById('sensor-start-btn').addEventListener('click', () => controlSensorData('start'));
    document.getElementById('sensor-stop-btn').addEventListener('click', () => controlSensorData('stop'));

    // Navigation commands
    const navStartBtn = document.getElementById('nav-start-btn');
    if (navStartBtn) navStartBtn.addEventListener('click', () => sendNavigationCommand('start'));
    const navPauseBtn = document.getElementById('nav-pause-btn');
    if (navPauseBtn) navPauseBtn.addEventListener('click', () => sendNavigationCommand('pause'));
    const navResumeBtn = document.getElementById('nav-resume-btn');
    if (navResumeBtn) navResumeBtn.addEventListener('click', () => sendNavigationCommand('resume'));
    const navStopBtn = document.getElementById('nav-stop-btn');
    if (navStopBtn) navStopBtn.addEventListener('click', () => sendNavigationCommand('stop'));

    const acceptHeadingBtn = document.getElementById('accept-heading-btn');
    if (acceptHeadingBtn) {
        acceptHeadingBtn.addEventListener('click', () => {
            if (socket && socket.connected) {
                socket.emit('instant_command', {
                    command: 'NAV_ACCEPT_HEADING',
                    payload: {},
                    device_id: state.deviceId
                });
                addLog('info', 'Instant command: NAV_ACCEPT_HEADING');
            }
        });
    }

    const sendWaypointsBtn = document.getElementById('send-waypoints-btn');
    if (sendWaypointsBtn) sendWaypointsBtn.addEventListener('click', sendWaypointsToRobot);

    // Line follower removed — no line follower hardware connected

    const manualButtonMap = {
        'manual-forward-btn': 'forward',
        'manual-left-btn': 'left',
        'manual-right-btn': 'right',
        'manual-reverse-btn': 'reverse',
        'manual-stop-btn': 'stop'
    };

    Object.entries(manualButtonMap).forEach(([elementId, direction]) => {
        const element = document.getElementById(elementId);
        if (!element) return;
        element.addEventListener('click', async () => {
            // Combine override + drive into a single instant command
            // to avoid double round-trip latency
            if (socket && socket.connected) {
                if (!state.manualOverride) {
                    socket.emit('instant_command', {
                        command: 'MANUAL_OVERRIDE',
                        payload: { mode: 'hold' },
                        device_id: state.deviceId
                    });
                    state.manualOverride = true;
                    updateControlIndicators();
                }
                await sendManualCommand(direction);
            } else {
                await requestManualMode();
                await sendManualCommand(direction);
            }
        });
    });

    const releaseBtn = document.getElementById('manual-release-btn');
    if (releaseBtn) releaseBtn.addEventListener('click', releaseManualMode);

    const speedSlider = document.getElementById('manual-speed-slider');
    if (speedSlider) {
        speedSlider.addEventListener('input', (event) => {
            updateManualSpeed(event.target.value);
        });
    }

    const autoSpeedSlider = document.getElementById('auto-speed-slider');
    if (autoSpeedSlider) {
        autoSpeedSlider.addEventListener('input', (event) => {
            updateAutoSpeed(event.target.value);
        });
    }

    const soundBuzzerBtn = document.getElementById('sound-buzzer-btn');
    if (soundBuzzerBtn) {
        soundBuzzerBtn.addEventListener('click', async () => {
            await sendRobotCommand('SOUND_BUZZER', { duration: 3 });
            addLog('info', 'Buzzer sounded for 3 seconds');
        });
    }

    // CC1101 Wireless control
    const engageWirelessBtn = document.getElementById('engage-wireless-btn');
    if (engageWirelessBtn) {
        engageWirelessBtn.addEventListener('click', async () => {
            const success = await sendRobotCommand('ENGAGE_WIRELESS', { engage: true });
            if (success) {
                const statusEl = document.getElementById('wireless-status');
                if (statusEl) {
                    statusEl.textContent = 'Status: Wireless Active (UNO Remote)';
                    statusEl.style.color = '#ff9800';
                }
                addLog('info', 'CC1101 backup control engaged');
            }
        });
    }

    const disengageWirelessBtn = document.getElementById('disengage-wireless-btn');
    if (disengageWirelessBtn) {
        disengageWirelessBtn.addEventListener('click', async () => {
            const success = await sendRobotCommand('ENGAGE_WIRELESS', { engage: false });
            if (success) {
                const statusEl = document.getElementById('wireless-status');
                if (statusEl) {
                    statusEl.textContent = 'Status: I2C Priority (Normal)';
                    statusEl.style.color = '#4CAF50';
                }
                addLog('info', 'CC1101 backup control disengaged');
            }
        });
    }

    const backupButtonMap = {
        'backup-forward-btn': 'forward',
        'backup-left-btn': 'left',
        'backup-right-btn': 'right',
        'backup-reverse-btn': 'reverse',
        'backup-stop-btn': 'stop'
    };

    Object.entries(backupButtonMap).forEach(([elementId, direction]) => {
        const element = document.getElementById(elementId);
        if (!element) return;
        element.addEventListener('click', async () => {
            await sendBackupDrive(direction);
        });
    });

    const backupEngageBtn = document.getElementById('backup-engage-btn');
    if (backupEngageBtn) {
        backupEngageBtn.addEventListener('click', () => sendBackupOverride('hold'));
    }

    const backupReleaseBtn = document.getElementById('backup-release-btn');
    if (backupReleaseBtn) {
        backupReleaseBtn.addEventListener('click', () => sendBackupOverride('release'));
    }

    const backupSpeedSlider = document.getElementById('backup-speed-slider');
    if (backupSpeedSlider) {
        backupSpeedSlider.addEventListener('input', (event) => {
            updateBackupSpeed(event.target.value);
        });
    }
}

/**
 * Export Sensor Data
 */
async function exportSensorData() {
    try {
        const response = await fetch(`${CONFIG.apiBaseUrl}/api/sensor_data?limit=1000`);
        const data = await response.json();
        
        if (data.status === 'success') {
            const csv = convertToCSV(data.data);
            downloadCSV(csv, `sensor_data_${new Date().toISOString()}.csv`);
            addLog('success', 'Data exported successfully');
        }
    } catch (error) {
        console.error('Error exporting data:', error);
        addLog('error', 'Failed to export data');
    }
}

/**
 * Convert to CSV
 */
function convertToCSV(data) {
    if (data.length === 0) return '';
    
    const headers = Object.keys(data[0]);
    const csvRows = [];
    
    csvRows.push(headers.join(','));
    
    for (const row of data) {
        const values = headers.map(header => {
            const val = row[header];
            return `"${val}"`;
        });
        csvRows.push(values.join(','));
    }
    
    return csvRows.join('\n');
}

/**
 * Download CSV
 */
function downloadCSV(csv, filename) {
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.setAttribute('href', url);
    a.setAttribute('download', filename);
    a.click();
}

function normalizeLocation(raw) {
    if (!raw) return null;

    const normalized = {};
    if (raw.latitude !== undefined && raw.latitude !== null) {
        normalized.latitude = Number(raw.latitude);
    }
    if (raw.longitude !== undefined && raw.longitude !== null) {
        normalized.longitude = Number(raw.longitude);
    }
    if (raw.heading !== undefined && raw.heading !== null) {
        normalized.heading = Number(raw.heading);
    }
    if (raw.speed !== undefined && raw.speed !== null) {
        normalized.speed = Number(raw.speed);
    }
    if (raw.altitude !== undefined && raw.altitude !== null) {
        normalized.altitude = Number(raw.altitude);
    }
    if (raw.satellites !== undefined && raw.satellites !== null) {
        normalized.satellites = Number(raw.satellites);
    }
    if (raw.timestamp) {
        normalized.timestamp = raw.timestamp;
    }
    if (raw.device_id) {
        normalized.device_id = raw.device_id;
    }
    if (raw.source) {
        normalized.source = raw.source;
    }
    return normalized;
}

// Initialize dashboard when DOM is ready
document.addEventListener('DOMContentLoaded', initDashboard);

// Clean up intervals when page is unloaded
window.addEventListener('beforeunload', () => {
    if (state.statusCheckInterval) {
        clearInterval(state.statusCheckInterval);
    }
});

// Make functions globally available
window.deleteWaypoint = deleteWaypoint;
window.addLog = addLog;

/* ─── Compass Widget Drawing Engine ─── */
const compassState = {
    heading: null,         // current robot heading (degrees)
    targetBearing: null,   // target waypoint bearing (degrees)
    headingError: null,    // signed error (degrees)
    acquired: false,       // heading acquired (error < 5°)
    navState: 'IDLE',
    distance: null,
    waypointIndex: null,
    waypointTotal: null
};

function drawCompass() {
    const canvas = document.getElementById('compass-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;
    const cx = W / 2;
    const cy = H / 2;
    const R = Math.min(cx, cy) - 16;

    ctx.clearRect(0, 0, W, H);

    // Outer ring
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, 2 * Math.PI);
    ctx.strokeStyle = '#d1d5db';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Inner ring
    ctx.beginPath();
    ctx.arc(cx, cy, R - 8, 0, 2 * Math.PI);
    ctx.strokeStyle = '#e5e7eb';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Cardinal labels and tick marks
    const cardinals = [
        { label: 'N', angle: 0, color: '#ef4444' },
        { label: 'E', angle: 90, color: '#6b7280' },
        { label: 'S', angle: 180, color: '#6b7280' },
        { label: 'W', angle: 270, color: '#6b7280' }
    ];
    ctx.font = 'bold 14px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    cardinals.forEach(c => {
        const rad = (c.angle - 90) * Math.PI / 180;
        const lx = cx + (R + 10) * Math.cos(rad);
        const ly = cy + (R + 10) * Math.sin(rad);
        ctx.fillStyle = c.color;
        ctx.fillText(c.label, lx, ly);
    });

    // Degree ticks every 30°
    for (let deg = 0; deg < 360; deg += 30) {
        const rad = (deg - 90) * Math.PI / 180;
        const inner = R - 8;
        const outer = R;
        ctx.beginPath();
        ctx.moveTo(cx + inner * Math.cos(rad), cy + inner * Math.sin(rad));
        ctx.lineTo(cx + outer * Math.cos(rad), cy + outer * Math.sin(rad));
        ctx.strokeStyle = '#9ca3af';
        ctx.lineWidth = deg % 90 === 0 ? 2 : 1;
        ctx.stroke();
    }

    // Small ticks every 10°
    for (let deg = 0; deg < 360; deg += 10) {
        if (deg % 30 === 0) continue;
        const rad = (deg - 90) * Math.PI / 180;
        const inner = R - 4;
        const outer = R;
        ctx.beginPath();
        ctx.moveTo(cx + inner * Math.cos(rad), cy + inner * Math.sin(rad));
        ctx.lineTo(cx + outer * Math.cos(rad), cy + outer * Math.sin(rad));
        ctx.strokeStyle = '#d1d5db';
        ctx.lineWidth = 0.5;
        ctx.stroke();
    }

    const needleLen = R - 20;

    // Determine if heading acquired (lines merge to orange)
    const acquired = compassState.acquired ||
        (compassState.heading !== null && compassState.targetBearing !== null &&
         Math.abs(compassState.headingError) < 5);

    // Draw TARGET BEARING line (red) — only when navigating
    if (compassState.targetBearing !== null) {
        const tRad = (compassState.targetBearing - 90) * Math.PI / 180;
        const color = acquired ? '#f59e0b' : '#ef4444';
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + needleLen * Math.cos(tRad), cy + needleLen * Math.sin(tRad));
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.setLineDash([6, 4]);
        ctx.stroke();
        ctx.setLineDash([]);

        // Target arrowhead
        const ax = cx + needleLen * Math.cos(tRad);
        const ay = cy + needleLen * Math.sin(tRad);
        ctx.beginPath();
        ctx.arc(ax, ay, 5, 0, 2 * Math.PI);
        ctx.fillStyle = color;
        ctx.fill();
    }

    // Draw HEADING line (blue) — always when available
    if (compassState.heading !== null) {
        const hRad = (compassState.heading - 90) * Math.PI / 180;
        const color = acquired ? '#f59e0b' : '#3b82f6';
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + needleLen * Math.cos(hRad), cy + needleLen * Math.sin(hRad));
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.stroke();

        // Heading arrowhead (triangle)
        const tipX = cx + needleLen * Math.cos(hRad);
        const tipY = cy + needleLen * Math.sin(hRad);
        const arrowSize = 10;
        ctx.beginPath();
        ctx.moveTo(tipX, tipY);
        ctx.lineTo(
            tipX - arrowSize * Math.cos(hRad - 0.4),
            tipY - arrowSize * Math.sin(hRad - 0.4)
        );
        ctx.lineTo(
            tipX - arrowSize * Math.cos(hRad + 0.4),
            tipY - arrowSize * Math.sin(hRad + 0.4)
        );
        ctx.closePath();
        ctx.fillStyle = color;
        ctx.fill();
    }

    // Center dot
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
    ctx.fillStyle = '#374151';
    ctx.fill();

    // Heading text in center
    ctx.font = 'bold 18px sans-serif';
    ctx.fillStyle = '#111827';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const hdgText = compassState.heading !== null ? `${Math.round(compassState.heading)}°` : '--°';
    ctx.fillText(hdgText, cx, cy + 30);
}

/** Map nav state string to CSS class and display label */
const NAV_STATE_MAP = {
    'IDLE':              { cls: 'nav-idle',       label: 'IDLE' },
    'ACQUIRING_HEADING': { cls: 'nav-acquiring',  label: 'ACQUIRING' },
    'HEADING_ACQUIRED':  { cls: 'nav-acquired',   label: 'ACQUIRED' },
    'NAVIGATING':        { cls: 'nav-navigating', label: 'NAVIGATING' },
    'OBSTACLE_DETECTED': { cls: 'nav-obstacle',   label: 'OBSTACLE' },
    'OBSTACLE_AVOID':    { cls: 'nav-obstacle',   label: 'AVOIDING' },
    'WAYPOINT_REACHED':  { cls: 'nav-reached',    label: 'REACHED' },
    'COMPLETE':          { cls: 'nav-complete',    label: 'COMPLETE' },
    'PAUSED':            { cls: 'nav-idle',        label: 'PAUSED' }
};

function updateNavStatusUI(nav) {
    if (!nav) return;

    // Update compass state — field names match nav_controller.get_status()
    if (nav.current_heading !== undefined && nav.current_heading !== null) {
        compassState.heading = Number(nav.current_heading);
    }
    if (nav.target_bearing !== undefined) {
        compassState.targetBearing = nav.target_bearing !== null ? Number(nav.target_bearing) : null;
    }
    if (nav.heading_error !== undefined) {
        compassState.headingError = nav.heading_error !== null ? Number(nav.heading_error) : null;
    }
    // Derive "acquired" from state name
    const st = (nav.state || 'IDLE').toUpperCase();
    compassState.acquired = (st === 'HEADING_ACQUIRED' || st === 'NAVIGATING' || st === 'WAYPOINT_REACHED');
    compassState.navState = nav.state || 'IDLE';
    // nav_controller sends distance_to_wp
    const dist = nav.distance_to_wp !== undefined ? nav.distance_to_wp : nav.distance;
    if (dist !== undefined) {
        compassState.distance = dist !== null ? Number(dist) : null;
    }
    // nav_controller sends current_wp_index / total_waypoints
    const wpIdx = nav.current_wp_index !== undefined ? nav.current_wp_index : nav.waypoint_index;
    if (wpIdx !== undefined) {
        compassState.waypointIndex = wpIdx;
    }
    const wpTotal = nav.total_waypoints !== undefined ? nav.total_waypoints : nav.waypoint_total;
    if (wpTotal !== undefined) {
        compassState.waypointTotal = wpTotal;
    }

    // Update text elements
    const stateKey = (nav.state || 'IDLE').toUpperCase();
    const mapped = NAV_STATE_MAP[stateKey] || { cls: 'nav-idle', label: stateKey };

    updateElement('nav-state-text', mapped.label);

    // State pill
    const pill = document.getElementById('nav-state-pill');
    if (pill) {
        pill.textContent = mapped.label;
        // Remove all nav-* classes then add current
        pill.className = 'status-badge ' + mapped.cls;
    }

    updateElement('nav-heading-text',
        compassState.heading !== null ? `${compassState.heading.toFixed(1)}°` : '--°');
    updateElement('nav-bearing-text',
        compassState.targetBearing !== null ? `${compassState.targetBearing.toFixed(1)}°` : '--°');
    updateElement('nav-error-text',
        compassState.headingError !== null ? `${compassState.headingError.toFixed(1)}°` : '--°');
    updateElement('nav-distance-text',
        compassState.distance !== null ? `${compassState.distance.toFixed(1)} m` : '-- m');

    if (compassState.waypointIndex !== null && compassState.waypointTotal !== null) {
        updateElement('nav-waypoint-text',
            `${compassState.waypointIndex + 1} / ${compassState.waypointTotal}`);
    } else {
        updateElement('nav-waypoint-text', '--');
    }

    // Also update navigationActive state for control indicator
    const navActive = stateKey !== 'IDLE' && stateKey !== 'COMPLETE';
    state.navigationActive = navActive;
    updateControlIndicators();

    // Enable accept-heading button only during heading acquisition
    const acceptBtn = document.getElementById('accept-heading-btn');
    if (acceptBtn) {
        acceptBtn.disabled = stateKey !== 'ACQUIRING_HEADING';
    }

    // Redraw compass
    drawCompass();
}

// Draw initial empty compass on load, with resize handler
document.addEventListener('DOMContentLoaded', () => {
    setTimeout(drawCompass, 100);
    // Redraw on window resize so compass scales if panel resizes
    window.addEventListener('resize', drawCompass);
});

/**
 * Handle robot events from backend
 */
function handleRobotEvent(event) {
    if (!event) return;
    const type = (event.type || 'EVENT').toUpperCase();
    const device = event.device_id || state.deviceId || 'robot';
    const payload = event.payload || {};

    // NAV_STATUS events: update compass + nav panel silently (no log/toast spam)
    if (type === 'NAV_STATUS' && payload.nav) {
        updateNavStatusUI(payload.nav);
        return;
    }

    // Rate limit UI spam for repeating obstacle events
    if (!state._obstacleUi) state._obstacleUi = { lastToastMs: 0 };

    // Simple severity mapping
    let level = 'info';
    if (type.includes('OBSTACLE') || payload.obstacle) level = 'warning';
    if (type.includes('ERROR')) level = 'error';

    // Build message
    let msg = `Event ${type} from ${device}`;
    const details = [];
    if (payload.distance !== undefined) details.push(`distance=${payload.distance}`);
    if (payload.distance_cm !== undefined) details.push(`distance=${payload.distance_cm}cm`);
    if (payload.direction) details.push(`direction=${payload.direction}`);
    if (payload.note) details.push(payload.note);
    if (details.length) msg += ` (${details.join(', ')})`;

    const nowMs = Date.now();
    const isObstacleEvent = (type === 'OBSTACLE_DETECTED');
    const toastIntervalMs = 5000;
    if (!isObstacleEvent || (nowMs - state._obstacleUi.lastToastMs) >= toastIntervalMs) {
        addLog(level, msg);
        showToast(level, 'Obstacle detected', msg);
        if (isObstacleEvent) state._obstacleUi.lastToastMs = nowMs;
    }

    // Show obstacle notification panel for OBSTACLE_DETECTED events
    if (type === 'OBSTACLE_DETECTED') {
        const notification = document.getElementById('obstacle-notification');
        const message = document.getElementById('obstacle-message');
        const distance = payload.distance_cm || payload.distance || 'unknown';
        message.textContent = `Obstacle detected at ${distance}cm`;
        notification.classList.remove('hidden');
        
        // Auto-hide after 10 seconds if no new event
        if (state._obstacleUi.hideTimer) clearTimeout(state._obstacleUi.hideTimer);
        state._obstacleUi.hideTimer = setTimeout(() => {
            notification.classList.add('hidden');
        }, 10000);
    }

    // Map visual indicator
    if (typeof window.showObstacleIndicator === 'function') {
        const dist = (payload.distance_cm !== undefined) ? Number(payload.distance_cm) : (payload.distance !== undefined ? Number(payload.distance) : NaN);
        window.showObstacleIndicator(isNaN(dist) ? null : dist, payload.direction || null);
    }
}

/**
 * Toast Notifications
 */
function showToast(type = 'info', title = '', detail = '') {
    const container = document.getElementById('toast-container');
    if (!container) return;
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.innerHTML = `
        <div>
            <div class="title">${title || type.toUpperCase()}</div>
            ${detail ? `<div class="detail">${detail}</div>` : ''}
        </div>
    `;
    container.appendChild(toast);
    // Auto-dismiss
    setTimeout(() => {
        toast.style.animation = 'fadeOut 200ms ease-out forwards';
        setTimeout(() => toast.remove(), 220);
    }, 4000);
}

/**
 * Overlay Windows (draggable/toggle)
 */
function initOverlaySystem() {
    const overlays = document.querySelectorAll('.overlay-window');
    let zCounter = 1000;

    // Persist/restore helpers
    const LAYOUT_KEY = 'dashboardLayout.v1';

    function loadLayout() {
        try {
            const raw = localStorage.getItem(LAYOUT_KEY);
            if (!raw) return {};
            return JSON.parse(raw) || {};
        } catch (e) {
            console.warn('Failed to load layout', e);
            return {};
        }
    }

    function saveLayout(layout) {
        try {
            localStorage.setItem(LAYOUT_KEY, JSON.stringify(layout));
        } catch (e) {
            console.warn('Failed to save layout', e);
        }
    }

    function getOverlayId(el) {
        return el.getAttribute('data-overlay-id');
    }

    function saveOverlayLayout(el, extra = {}) {
        const id = getOverlayId(el);
        if (!id) return;
        const rect = el.getBoundingClientRect();
        const layout = loadLayout();
        layout.overlays = layout.overlays || {};
        layout.overlays[id] = {
            left: Math.max(0, Math.round(rect.left)),
            top: Math.max(0, Math.round(rect.top)),
            width: Math.round(rect.width),
            height: Math.round(rect.height),
            hidden: el.classList.contains('hidden'),
            zIndex: Number(el.style.zIndex || 1000),
            ...extra
        };
        saveLayout(layout);
    }

    function applySavedLayout(el) {
        const id = getOverlayId(el);
        if (!id) return;
        const layout = loadLayout();
        const conf = layout.overlays && layout.overlays[id];
        if (!conf) return;
        el.style.left = `${conf.left}px`;
        el.style.top = `${conf.top}px`;
        el.style.right = 'auto';
        el.style.bottom = 'auto';
        if (conf.width) el.style.width = `${conf.width}px`;
        if (conf.height) el.style.height = `${conf.height}px`;
        if (conf.zIndex) el.style.zIndex = String(conf.zIndex);
        if (conf.hidden) el.classList.add('hidden'); else el.classList.remove('hidden');
    }

    // Apply saved layout first
    overlays.forEach(applySavedLayout);

    overlays.forEach(overlay => {
        // Bring to front when interacted
        overlay.addEventListener('mousedown', () => {
            zCounter += 1;
            overlay.style.zIndex = String(zCounter);
            saveOverlayLayout(overlay);
        });

        // Hook close buttons
        overlay.querySelectorAll('.overlay-close').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const id = btn.getAttribute('data-overlay-target');
                if (id) toggleOverlay(id, false);
                e.stopPropagation();
            });
        });

        // Draggable by header
        const header = overlay.querySelector('.overlay-header');
        if (!header) return;
        let dragging = false;
        let startX = 0, startY = 0;
        let startLeft = 0, startTop = 0;

        header.addEventListener('mousedown', (e) => {
            dragging = true;
            overlay.style.transition = 'none';
            startX = e.clientX;
            startY = e.clientY;
            const rect = overlay.getBoundingClientRect();
            startLeft = rect.left;
            startTop = rect.top;
            document.body.classList.add('dragging-overlay');
            e.preventDefault();
        });

        document.addEventListener('mousemove', (e) => {
            if (!dragging) return;
            const dx = e.clientX - startX;
            const dy = e.clientY - startY;
            overlay.style.left = `${Math.max(0, startLeft + dx)}px`;
            overlay.style.top = `${Math.max(0, startTop + dy)}px`;
            overlay.style.right = 'auto';
            overlay.style.bottom = 'auto';
        });

        document.addEventListener('mouseup', () => {
            if (dragging) {
                dragging = false;
                document.body.classList.remove('dragging-overlay');
                saveOverlayLayout(overlay);
            }
        });

        // Resizable via corner handle
        const handle = overlay.querySelector('.resize-handle');
        if (handle) {
            let resizing = false;
            let startW = 0, startH = 0;
            handle.addEventListener('mousedown', (e) => {
                resizing = true;
                zCounter += 1;
                overlay.style.zIndex = String(zCounter);
                const rect = overlay.getBoundingClientRect();
                startW = rect.width;
                startH = rect.height;
                startX = e.clientX;
                startY = e.clientY;
                document.body.classList.add('dragging-overlay');
                e.preventDefault();
                e.stopPropagation();
            });

            document.addEventListener('mousemove', (e) => {
                if (!resizing) return;
                const dx = e.clientX - startX;
                const dy = e.clientY - startY;
                const newW = Math.max(360, startW + dx);
                const newH = Math.max(240, startH + dy);
                overlay.style.width = `${newW}px`;
                overlay.style.height = `${newH}px`;
            });

            document.addEventListener('mouseup', () => {
                if (resizing) {
                    resizing = false;
                    document.body.classList.remove('dragging-overlay');
                    saveOverlayLayout(overlay);
                }
            });
        }
    });

    // Expose reset for toolbar
    window.__resetOverlayLayout = () => {
        localStorage.removeItem(LAYOUT_KEY);
        // Reload to return to default inline styles
        window.location.reload();
    };
}

function getOverlay(id) {
    return document.querySelector(`.overlay-window[data-overlay-id="${id}"]`);
}

function toggleOverlay(id, forceOpen = undefined) {
    const el = getOverlay(id);
    if (!el) return;
    const shouldOpen = forceOpen !== undefined ? forceOpen : el.classList.contains('hidden');
    if (shouldOpen) {
        el.classList.remove('hidden');
        // Raise z-index a bit on open
        el.style.zIndex = String(1100);
        // Persist visible state and position
        const rect = el.getBoundingClientRect();
        el.style.left = `${Math.max(0, Math.round(rect.left))}px`;
        el.style.top = `${Math.max(0, Math.round(rect.top))}px`;
        el.style.right = 'auto';
        el.style.bottom = 'auto';
    } else {
        el.classList.add('hidden');
    }
    // Save new visibility state
    try {
        const LAYOUT_KEY = 'dashboardLayout.v1';
        const layout = JSON.parse(localStorage.getItem(LAYOUT_KEY) || '{}');
        layout.overlays = layout.overlays || {};
        const rect = el.getBoundingClientRect();
        layout.overlays[id] = {
            left: Math.max(0, Math.round(rect.left)),
            top: Math.max(0, Math.round(rect.top)),
            width: Math.round(rect.width),
            height: Math.round(rect.height),
            hidden: el.classList.contains('hidden'),
            zIndex: Number(el.style.zIndex || 1100)
        };
        localStorage.setItem(LAYOUT_KEY, JSON.stringify(layout));
    } catch (e) {
        console.warn('Failed to persist overlay state', e);
    }
}

// Expose for debugging if needed
window.toggleOverlay = toggleOverlay;

/**
 * Zoom Controls
 */
// ── Camera zoom controls ──────────────────────────────────────
(function initCameraZoom() {
    const MIN = 1.0, MAX = 4.0, STEP = 0.5;
    let camZoom = 1.0;

    function apply() {
        const feed = document.getElementById('camera-feed');
        if (feed) feed.style.transform = `scale(${camZoom})`;
        const label = document.getElementById('cam-zoom-level');
        if (label) label.textContent = `${camZoom}×`;
    }

    document.addEventListener('DOMContentLoaded', () => {
        const inBtn = document.getElementById('cam-zoom-in');
        const outBtn = document.getElementById('cam-zoom-out');

        if (inBtn) inBtn.addEventListener('click', () => {
            camZoom = Math.min(MAX, +(camZoom + STEP).toFixed(1));
            apply();
        });
        if (outBtn) outBtn.addEventListener('click', () => {
            camZoom = Math.max(MIN, +(camZoom - STEP).toFixed(1));
            apply();
        });
    });
})();

function initZoomControls() {
    const outBtn = document.getElementById('zoom-out-btn');
    const inBtn = document.getElementById('zoom-in-btn');
    const resetBtn = document.getElementById('zoom-reset-btn');
    state.zoomScale = 1.0;
    state.minZoom = 0.75;
    state.maxZoom = 1.5;
    state.zoomStep = 0.1;

    function applyZoom() {
        const wrapper = document.getElementById('dashboard-scale');
        if (wrapper) {
            wrapper.style.transform = `scale(${state.zoomScale})`;
        }
        // Update label and persist
        const resetBtnEl = document.getElementById('zoom-reset-btn');
        if (resetBtnEl) {
            resetBtnEl.textContent = `${Math.round(state.zoomScale * 100)}%`;
        }
        try {
            const LAYOUT_KEY = 'dashboardLayout.v1';
            const layout = JSON.parse(localStorage.getItem(LAYOUT_KEY) || '{}');
            layout.zoom = state.zoomScale;
            localStorage.setItem(LAYOUT_KEY, JSON.stringify(layout));
        } catch (e) {
            // ignore
        }
    }

    // Load saved zoom first
    try {
        const saved = JSON.parse(localStorage.getItem('dashboardLayout.v1') || '{}');
        if (saved && typeof saved.zoom === 'number') {
            state.zoomScale = Math.min(state.maxZoom, Math.max(state.minZoom, saved.zoom));
        }
    } catch (e) { /* ignore */ }

    if (outBtn) outBtn.addEventListener('click', () => {
        state.zoomScale = Math.max(state.minZoom, +(state.zoomScale - state.zoomStep).toFixed(2));
        applyZoom();
    });
    if (inBtn) inBtn.addEventListener('click', () => {
        state.zoomScale = Math.min(state.maxZoom, +(state.zoomScale + state.zoomStep).toFixed(2));
        applyZoom();
    });
    if (resetBtn) resetBtn.addEventListener('click', () => {
        state.zoomScale = 1.0;
        applyZoom();
    });

    applyZoom();
}
