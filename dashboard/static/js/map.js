/**
 * Map Module - Handles GPS visualization and waypoint management
 */

let map;
let robotMarker;
let waypointMarkers = [];
let robotPath;
let mapClickEnabled = false;
let obstacleCircles = [];

/**
 * Initialize Map
 */
function initMap() {
    // Create map centered at default location
    const defaultCenter = (window.CONFIG?.map?.defaultCenter) || [0, 0];
    const defaultZoom = window.CONFIG?.map?.defaultZoom || 2;
    map = L.map('map').setView(defaultCenter, defaultZoom);
    
    // Add OpenStreetMap tiles
    const tileUrl = window.CONFIG?.map?.tileUrl || 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';
    const attribution = window.CONFIG?.map?.attribution || '© OpenStreetMap contributors';
    const tileOptions = {
        attribution,
        maxZoom: 19
    };

    const apiKey = window.CONFIG?.map?.apiKey;
    if (apiKey) {
        tileOptions.apiKey = apiKey;
    }

    L.tileLayer(tileUrl, tileOptions).addTo(map);
    
    // Create robot marker (blue dot)
    const robotIcon = L.divIcon({
        className: 'robot-marker',
        html: '<div class="robot-marker-body" style="background-color: #3b82f6; border: 2px solid white; box-shadow: 0 0 10px rgba(59, 130, 246, 0.5);"></div>',
        iconSize: [16, 16],
        iconAnchor: [8, 8]
    });
    
    robotMarker = L.marker([0, 0], { icon: robotIcon }).addTo(map);
    robotMarker.bindPopup('Robot Position');
    
    // Create path polyline
    robotPath = L.polyline([], {
        color: 'blue',
        weight: 3,
        opacity: 0.7
    }).addTo(map);
    
    // Map click listener for adding waypoints
    map.on('click', function(e) {
        if (mapClickEnabled) {
            addWaypointFromMap(e.latlng.lat, e.latlng.lng);
        }
    });
    
    // Add scale control
    L.control.scale().addTo(map);
    
    // Add custom controls
    addMapControls();
    
    window.addLog('info', 'Map initialized');
}

/**
 * Update Robot Position
 */
function updateRobotPosition(lat, lon, heading = 0, source = 'primary') {
    if (!map || !robotMarker) return;
    
    const newPos = [lat, lon];
    
    // Update marker position
    robotMarker.setLatLng(newPos);
    
    const markerElement = robotMarker.getElement();
    if (markerElement) {
        markerElement.classList.toggle('backup-mode', source === 'backup');
        const body = markerElement.querySelector('.robot-marker-body');
        if (body) {
            body.style.transform = `rotate(${heading}deg)`;
        }
    }
    
    // Update path
    const pathCoords = robotPath.getLatLngs();
    pathCoords.push(newPos);
    
    // Keep only last 100 points
    if (pathCoords.length > 100) {
        pathCoords.shift();
    }
    
    robotPath.setLatLngs(pathCoords);
    
    // Update popup
    robotMarker.setPopupContent(`
        <b>Robot Position</b><br>
        Lat: ${lat.toFixed(6)}<br>
        Lon: ${lon.toFixed(6)}<br>
        Heading: ${heading.toFixed(1)}°<br>
        Source: ${source === 'backup' ? 'Backup (LoRa)' : 'Primary'}
    `);
}

/**
 * Show visual indicator near the robot for obstacles
 * @param {number|null} distanceCm - distance in centimeters (optional)
 * @param {string|null} direction - optional direction label
 */
function showObstacleIndicator(distanceCm = null, direction = null) {
    if (!map || !robotMarker) return;
    const pos = robotMarker.getLatLng();
    const radiusMeters = (typeof distanceCm === 'number' && !isNaN(distanceCm)) ? Math.max(0.5, distanceCm / 100.0) : 1.5;
    const circle = L.circle(pos, {
        radius: radiusMeters,
        color: '#ef4444',
        fillColor: '#ef4444',
        fillOpacity: 0.15,
        weight: 2
    }).addTo(map);
    obstacleCircles.push(circle);

    // Brief glow on robot marker
    const markerElement = robotMarker.getElement();
    const body = markerElement && markerElement.querySelector('.robot-marker-body');
    if (body) {
        body.classList.add('obstacle-glow');
        setTimeout(() => body.classList.remove('obstacle-glow'), 1600);
    }

    // Auto-remove the circle after a few seconds
    setTimeout(() => {
        try {
            map.removeLayer(circle);
        } catch (e) { /* ignore */ }
        obstacleCircles = obstacleCircles.filter(c => c !== circle);
    }, 3000);
}

/**
 * Center Map on Robot
 */
function centerMapOnRobot() {
    if (!robotMarker) return;
    
    const pos = robotMarker.getLatLng();
    map.setView(pos, 16, { animate: true });
    
    window.addLog('info', 'Map centered on robot');
}

/**
 * Update Map Waypoints
 */
function updateMapWaypoints(waypoints) {
    // Clear existing waypoint markers
    waypointMarkers.forEach(marker => map.removeLayer(marker));
    waypointMarkers = [];
    
    // Clear existing pathway lines
    if (window.robotToWaypointLine) {
        map.removeLayer(window.robotToWaypointLine);
    }
    if (window.waypointPathLine) {
        map.removeLayer(window.waypointPathLine);
    }
    
    // Add new waypoint markers
    waypoints.forEach((wp, index) => {
        const waypointIcon = L.divIcon({
            className: 'waypoint-marker',
            html: `<div style="
                background: #ef4444;
                color: white;
                border-radius: 50%;
                width: 30px;
                height: 30px;
                display: flex;
                align-items: center;
                justify-content: center;
                font-weight: bold;
                border: 3px solid white;
                box-shadow: 0 2px 5px rgba(0,0,0,0.3);
            ">${wp.sequence}</div>`,
            iconSize: [30, 30],
            iconAnchor: [15, 15]
        });
        
        const marker = L.marker([wp.latitude, wp.longitude], {
            icon: waypointIcon
        }).addTo(map);
        
        marker.bindPopup(`
            <b>Waypoint #${wp.sequence}</b><br>
            ${wp.description || 'No description'}<br>
            Lat: ${wp.latitude.toFixed(6)}<br>
            Lon: ${wp.longitude.toFixed(6)}
        `);
        
        waypointMarkers.push(marker);
    });
    
    // Draw pathway from robot to first waypoint and between waypoints
    if (waypoints.length > 0 && robotMarker) {
        const robotPos = robotMarker.getLatLng();
        const firstWaypoint = waypoints[0];
        
        // Line from robot to first waypoint (blue solid line)
        window.robotToWaypointLine = L.polyline([
            [robotPos.lat, robotPos.lng],
            [firstWaypoint.latitude, firstWaypoint.longitude]
        ], {
            color: '#3b82f6',
            weight: 3,
            opacity: 0.8
        }).addTo(map);
        
        // Line connecting waypoints (red dashed line)
        if (waypoints.length > 1) {
            const waypointCoords = waypoints.map(wp => [wp.latitude, wp.longitude]);
            window.waypointPathLine = L.polyline(waypointCoords, {
                color: '#ef4444',
                weight: 2,
                opacity: 0.7,
                dashArray: '10, 10'
            }).addTo(map);
        }
    }
    
    // Fit bounds to show all waypoints and robot
    if (waypoints.length > 0) {
        const bounds = L.latLngBounds([...waypoints.map(wp => [wp.latitude, wp.longitude])]);
        if (robotMarker) {
            bounds.extend(robotMarker.getLatLng());
        }
        map.fitBounds(bounds, { padding: [50, 50] });
    }
}

/**
 * Add Waypoint from Map Click
 */
function addWaypointFromMap(lat, lon) {
    const description = prompt('Enter waypoint description (optional):');
    
    if (description !== null) { // null means cancelled
        window.addWaypoint(lat, lon, description || `Map Waypoint`);
        toggleWaypointMode(false);
    }
}

/**
 * Toggle Waypoint Click Mode
 */
function toggleWaypointMode(enable) {
    mapClickEnabled = enable;
    
    const btn = document.getElementById('add-waypoint-btn');
    if (enable) {
        btn.textContent = '❌ Cancel';
        btn.classList.add('btn-primary');
        map.getContainer().style.cursor = 'crosshair';
        window.addLog('info', 'Click on map to add waypoint');
    } else {
        btn.textContent = '📍 Add Waypoint';
        btn.classList.remove('btn-primary');
        map.getContainer().style.cursor = '';
    }
}

/**
 * Add Map Controls
 */
function addMapControls() {
    // Center robot button
    document.getElementById('center-robot-btn').addEventListener('click', centerMapOnRobot);
    
    // Add waypoint button
    document.getElementById('add-waypoint-btn').addEventListener('click', () => {
        toggleWaypointMode(!mapClickEnabled);
    });
}

/**
 * Load GPS Track History
 */
async function loadGPSTrack() {
    try {
        const response = await fetch(`${window.CONFIG.apiBaseUrl}/api/gps_data/track?limit=100`);
        const data = await response.json();
        
        if (data.status === 'success' && data.data.length > 0) {
            const trackCoords = data.data.map(point => [point.latitude, point.longitude]);
            robotPath.setLatLngs(trackCoords);
            
            // Update robot position to latest
            const latest = data.data[0];
            updateRobotPosition(latest.latitude, latest.longitude, latest.heading, latest.source || 'primary');
            
            // Center map on track
            map.fitBounds(robotPath.getBounds(), { padding: [50, 50] });
        }
    } catch (error) {
        console.error('Error loading GPS track:', error);
    }
}

function setBackupMapMode(active) {
    const mapElement = document.getElementById('map');
    if (!mapElement) return;
    mapElement.classList.toggle('backup-active', Boolean(active));
}

// Initialize map when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    // Wait a bit for the map container to be ready
    setTimeout(initMap, 100);
    
    // Load GPS track history
    setTimeout(loadGPSTrack, 1000);
});

// Export functions to global scope
window.updateRobotPosition = updateRobotPosition;
window.updateMapWaypoints = updateMapWaypoints;
window.centerMapOnRobot = centerMapOnRobot;
window.setBackupMapMode = setBackupMapMode;
window.showObstacleIndicator = showObstacleIndicator;
