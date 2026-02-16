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

send_at() {
    # send_at <command> [<wait_seconds>]
    local cmd="$1"
    local wait="${2:-$TIMEOUT}"
    local resp

    # Use a file descriptor so we don't re-open the port each time
    resp=$(timeout "$wait" bash -c "
        exec 3<>\"$AT_PORT\"
        stty -F \"$AT_PORT\" $BAUD raw -echo -echoe -echok
        printf '%s\r\n' '$cmd' >&3
        sleep 0.3
        cat <&3 &
        CAT_PID=\$!
        sleep $wait
        kill \$CAT_PID 2>/dev/null || true
        exec 3>&-
    " 2>/dev/null || true)

    echo "$resp"
}

send_at_clean() {
    # Send AT command and capture response cleanly
    local cmd="$1"
    local wait="${2:-$TIMEOUT}"

    stty -F "$AT_PORT" "$BAUD" raw -echo -echoe -echok 2>/dev/null || true

    {
        printf '%s\r\n' "$cmd"
        sleep "$wait"
    } > "$AT_PORT" &
    local WRITE_PID=$!

    local resp=""
    resp=$(timeout "$((wait + 1))" cat "$AT_PORT" 2>/dev/null || true)
    wait "$WRITE_PID" 2>/dev/null || true

    echo "$resp"
}

# Cleaner AT send using stty + direct read
at_cmd() {
    local cmd="$1"
    local wait="${2:-$TIMEOUT}"
    local resp=""

    # Configure port
    stty -F "$AT_PORT" "$BAUD" raw -echo -echoe -echok 2>/dev/null || die "Cannot configure $AT_PORT"

    # Flush input
    timeout 0.2 cat "$AT_PORT" > /dev/null 2>&1 || true

    # Send command
    printf '%s\r\n' "$cmd" > "$AT_PORT"

    # Read response
    local end_time=$(( $(date +%s) + wait + 1 ))
    while [ "$(date +%s)" -lt "$end_time" ]; do
        local chunk
        chunk=$(timeout 1 head -c 512 "$AT_PORT" 2>/dev/null || true)
        resp+="$chunk"
        if echo "$resp" | grep -qE '(OK|ERROR|\+CME ERROR)'; then
            break
        fi
    done

    echo "$resp"
}

# ---- pre-flight checks -------------------------------------
[ "$(id -u)" -eq 0 ] || die "Run as root:  sudo $0"
[ -e "$AT_PORT" ]    || die "AT port $AT_PORT not found. Is SIM7600E connected via USB?"

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
