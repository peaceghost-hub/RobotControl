#!/bin/bash
# ============================================================
#  enable_sim7600e_gps.sh — Enable GPS/GNSS on SIM7600E module
# ============================================================
#  SIM7600E USB port layout (Qualcomm QMI):
#    ttyUSB0 = DIAG / debug
#    ttyUSB1 = GPS NMEA stream (once GPS is enabled)
#    ttyUSB2 = AT command port
#    ttyUSB3 = PPP / data
#
#  Usage:
#    sudo bash enable_sim7600e_gps.sh          # enable GPS and show fix
#    sudo bash enable_sim7600e_gps.sh --nmea   # also tail NMEA on ttyUSB1
#    sudo bash enable_sim7600e_gps.sh --off    # turn GPS off
# ============================================================

set -euo pipefail

AT_PORT="${SIM_AT_PORT:-/dev/ttyUSB2}"
NMEA_PORT="${SIM_NMEA_PORT:-/dev/ttyUSB1}"
BAUD=115200
TIMEOUT=2

RED='\033[0;31m'
GRN='\033[0;32m'
YEL='\033[1;33m'
CYN='\033[0;36m'
RST='\033[0m'

# ---- helpers ------------------------------------------------
die()  { echo -e "${RED}ERROR: $*${RST}" >&2; exit 1; }
info() { echo -e "${CYN}>> $*${RST}"; }
ok()   { echo -e "${GRN}✓  $*${RST}"; }
warn() { echo -e "${YEL}⚠  $*${RST}"; }

# ---- AT command via python3 + pyserial ----------------------
# Much more reliable than stty + shell redirects on USB-serial.
at_cmd() {
    local cmd="$1"
    local wait="${2:-$TIMEOUT}"
    python3 -c "
import serial, sys, time
try:
    ser = serial.Serial('$AT_PORT', $BAUD, timeout=0.5, write_timeout=1,
                        rtscts=False, dsrdtr=False)
except Exception as e:
    print(f'__PORT_ERROR__:{e}', file=sys.stderr)
    sys.exit(1)
try:
    ser.reset_input_buffer()
    ser.write(b'${cmd}\r\n')
    resp = b''
    deadline = time.time() + float($wait)
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            resp += ser.read(n)
            if b'OK' in resp or b'ERROR' in resp:
                break
        time.sleep(0.05)
    print(resp.decode('utf-8', errors='ignore'))
finally:
    ser.close()
" 2>&1
}

# ---- pre-flight checks -------------------------------------
[ "$(id -u)" -eq 0 ] || die "Run as root:  sudo $0"
[ -e "$AT_PORT" ]    || die "AT port $AT_PORT not found. Is SIM7600E connected via USB?"

# Check python3 + pyserial are available
python3 -c 'import serial' 2>/dev/null || die "pyserial not installed. Run: pip3 install pyserial"

# Check if ModemManager or another process is holding the port
if command -v fuser &>/dev/null; then
    PIDS=$(fuser "$AT_PORT" 2>/dev/null || true)
    if [ -n "$PIDS" ]; then
        PROCS=$(ps -p ${PIDS// /,} -o comm= 2>/dev/null || echo "unknown")
        warn "Port $AT_PORT is held by: $PROCS (PIDs: $PIDS)"
        if echo "$PROCS" | grep -qi 'ModemManager'; then
            info "Stopping ModemManager to release the port..."
            systemctl stop ModemManager 2>/dev/null || true
            sleep 1
            ok "ModemManager stopped"
        else
            warn "Another process holds the port — will try anyway"
        fi
    fi
fi

# ---- parse args ---------------------------------------------
MODE="enable"
for arg in "$@"; do
    case "$arg" in
        --off|--disable) MODE="disable" ;;
        --nmea|--stream) MODE="nmea"    ;;
        --help|-h)
            echo "Usage: sudo $0 [--nmea | --off]"
            echo "  (no args)  Enable GPS and poll for a fix"
            echo "  --nmea     Enable GPS and stream NMEA sentences from $NMEA_PORT"
            echo "  --off      Turn GPS off"
            exit 0
            ;;
    esac
done

# ---- talk to module -----------------------------------------
info "Using AT port: $AT_PORT"

# Quick AT check
info "Checking module responds..."
RESP=$(at_cmd "AT" 2)
if echo "$RESP" | grep -q "OK"; then
    ok "Module alive"
else
    die "No response from $AT_PORT — check USB cable and that no other process holds the port"
fi

# Show module identity
info "Module info:"
at_cmd "ATI" 2 | grep -vE '^\s*$|^AT'

# ---- GPS OFF ------------------------------------------------
if [ "$MODE" = "disable" ]; then
    info "Turning GPS OFF..."
    RESP=$(at_cmd "AT+CGPS=0" 3)
    if echo "$RESP" | grep -q "OK"; then
        ok "GPS disabled"
    else
        warn "GPS may already be off, or error: $RESP"
    fi
    exit 0
fi

# ---- GPS ON -------------------------------------------------
info "Enabling GPS (AT+CGPS=1,1) ..."
RESP=$(at_cmd "AT+CGPS=1,1" 3)
if echo "$RESP" | grep -q "OK"; then
    ok "GPS engine started (standalone mode)"
elif echo "$RESP" | grep -q "GPS has started"; then
    ok "GPS was already running"
else
    warn "Response: $RESP"
    warn "Trying to stop and restart GPS..."
    at_cmd "AT+CGPS=0" 2 > /dev/null
    sleep 1
    RESP=$(at_cmd "AT+CGPS=1,1" 3)
    if echo "$RESP" | grep -q "OK"; then
        ok "GPS engine started after reset"
    else
        die "Cannot start GPS: $RESP"
    fi
fi

# Enable NMEA output on the NMEA USB port (ttyUSB1)
info "Enabling NMEA output on USB NMEA port..."
at_cmd "AT+CGPSINFO=0" 2 > /dev/null 2>&1   # stop unsolicited info on AT port
at_cmd "AT+CGNSSPORTSWITCH=0,1" 2 > /dev/null 2>&1  # NMEA → USB1 (some firmware versions)

# Set NMEA sentence types: GGA, RMC, GSV, GSA
at_cmd "AT+CGPSNMEA=200191" 2 > /dev/null 2>&1

# ---- NMEA stream mode ---------------------------------------
if [ "$MODE" = "nmea" ]; then
    info "Streaming NMEA from $NMEA_PORT (Ctrl+C to stop)..."
    [ -e "$NMEA_PORT" ] || die "NMEA port $NMEA_PORT not found"
    stty -F "$NMEA_PORT" 115200 raw -echo 2>/dev/null || true
    echo ""
    cat "$NMEA_PORT"
    exit 0
fi

# ---- poll for fix -------------------------------------------
info "Waiting for GPS fix (can take 30-90 s cold start, outdoor with clear sky)..."
echo ""

FIX_COUNT=0
MAX_POLLS=60   # ~2 minutes

for i in $(seq 1 $MAX_POLLS); do
    RESP=$(at_cmd "AT+CGPSINFO" 2)

    # +CGPSINFO: lat,N/S,lon,E/W,date,UTC,alt,speed,course
    # If no fix:  +CGPSINFO: ,,,,,,,,
    CGPS_LINE=$(echo "$RESP" | grep '+CGPSINFO:' | head -1)

    if [ -z "$CGPS_LINE" ]; then
        printf "\r  [%02d/%d] No response yet...            " "$i" "$MAX_POLLS"
        sleep 2
        continue
    fi

    # Extract fields after the colon
    DATA="${CGPS_LINE#*: }"

    # Check if we have actual data (not all commas)
    if echo "$DATA" | grep -qE '^,+$'; then
        printf "\r  [%02d/%d] Searching for satellites...     " "$i" "$MAX_POLLS"
        sleep 2
        continue
    fi

    # We have a fix!
    FIX_COUNT=$((FIX_COUNT + 1))

    # Parse fields: lat,N/S,lon,E/W,date,time,alt,speed,course
    IFS=',' read -r LAT NS LON EW DATE UTC ALT SPEED COURSE <<< "$DATA"

    if [ -n "$LAT" ] && [ -n "$LON" ]; then
        echo ""
        echo ""
        ok "GPS FIX ACQUIRED!"
        echo "  ──────────────────────────────────────"
        echo -e "  Latitude  : ${GRN}${LAT} ${NS}${RST}"
        echo -e "  Longitude : ${GRN}${LON} ${EW}${RST}"
        echo -e "  Altitude  : ${ALT:-N/A} m"
        echo -e "  Speed     : ${SPEED:-N/A} knots"
        echo -e "  Course    : ${COURSE:-N/A}°"
        echo -e "  UTC Time  : ${UTC:-N/A}"
        echo -e "  Date      : ${DATE:-N/A}"
        echo "  ──────────────────────────────────────"

        # Convert DDMM.MMMM → DD.DDDDDD for convenience
        if [ -n "$LAT" ] && [ -n "$LON" ]; then
            DEG_LAT=$(echo "$LAT" | awk '{d=int($1/100); m=$1-d*100; printf "%.6f", d+m/60}')
            DEG_LON=$(echo "$LON" | awk '{d=int($1/100); m=$1-d*100; printf "%.6f", d+m/60}')
            [ "$NS" = "S" ] && DEG_LAT="-$DEG_LAT"
            [ "$EW" = "W" ] && DEG_LON="-$DEG_LON"
            echo ""
            echo -e "  Decimal   : ${GRN}${DEG_LAT}, ${DEG_LON}${RST}"
            echo -e "  Maps      : https://maps.google.com/?q=${DEG_LAT},${DEG_LON}"
        fi
        echo ""
        info "GPS is running. NMEA sentences available on $NMEA_PORT"
        info "To stream: sudo $0 --nmea"
        info "To stop:   sudo $0 --off"
        exit 0
    fi

    sleep 2
done

echo ""
warn "No fix after $MAX_POLLS polls (~$((MAX_POLLS * 2))s)."
echo "  Tips:"
echo "  • Make sure the GPS antenna is connected to the GNSS port (not LTE)"
echo "  • Go outdoors with clear sky view"
echo "  • Cold start can take 60-120 seconds"
echo "  • Check antenna cable and connector"
echo ""
info "GPS engine is still running. You can:"
echo "  • Wait longer and re-run:  sudo $0"
echo "  • Stream NMEA:             sudo $0 --nmea"
echo "  • Turn off:                sudo $0 --off"
exit 1
