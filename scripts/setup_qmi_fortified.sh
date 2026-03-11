#!/usr/bin/env bash
# ============================================================
#  Fortified QMI Auto-Connect for SIM7600E  (v2 — resilient)
#
#  Replaces the basic setup_qmi_autoconnect.sh with:
#   • Kernel module auto-loading  (qmi_wwan, cdc_wdm)
#   • USB device recovery         (rebind if wwan0 vanishes)
#   • DNS hardening               (force public nameservers)
#   • Multi-layer health checks   (iface → IP → ping → DNS → HTTP)
#   • Lock-file guarded reconnect (no double-connect race)
#   • 30 s watchdog cycle with escalating recovery
#
#  Usage:
#    sudo ./setup_qmi_fortified.sh [APN]
#
#  Example:
#    sudo ./setup_qmi_fortified.sh Econet
# ============================================================

set -euo pipefail

APN="${1:-Econet}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Run with sudo"
    exit 1
fi

echo "==========================================="
echo " Fortified QMI Auto-Connect (APN: $APN)"
echo "==========================================="
echo ""

# ---- Install prerequisites ----
echo "[1/5] Installing prerequisites..."
apt-get update -qq
apt-get install -y libqmi-utils udhcpc usb-modeswitch 2>/dev/null || true

# Ensure kernel modules are loaded on every boot
echo "[2/5] Configuring kernel modules..."
for mod in qmi_wwan cdc_wdm option; do
    modprobe "$mod" 2>/dev/null || true
    grep -qxF "$mod" /etc/modules 2>/dev/null || echo "$mod" >> /etc/modules
done

# ===================================================================
#  Deploy /usr/local/bin/qmi-connect.sh  (fortified connection script)
# ===================================================================
echo "[3/5] Installing connection script..."

cat > /usr/local/bin/qmi-connect.sh << 'CONNECTEOF'
#!/usr/bin/env bash
# ── Fortified QMI connection manager for SIM7600E ──────────────
# Called by systemd service or watchdog.  Handles:
#   • Kernel module loading
#   • USB rebind when /dev/cdc-wdm* missing but lsusb sees modem
#   • wwan0 bring-up with raw_ip mode
#   • QMI data session start
#   • DHCP + DNS hardening
#   • Route metric management
# ────────────────────────────────────────────────────────────────

APN="__APN__"
IFACE="wwan0"
QMI_DEVICE=""
MAX_RETRIES=5
LOCKFILE="/tmp/qmi-connect.lock"
DNS_SERVERS="8.8.8.8 1.1.1.1 8.8.4.4"

log() { echo "$(date '+%Y-%m-%d %H:%M:%S') [qmi-connect] $*" | tee -a /var/log/qmi-connect.log; }

# ── Prevent concurrent runs ────────────────────────────────────
exec 200>"$LOCKFILE"
if ! flock -n 200; then
    log "Another qmi-connect instance is running — skipping."
    exit 0
fi

# ── Helper: find QMI character device ──────────────────────────
find_qmi_dev() {
    for dev in /dev/cdc-wdm0 /dev/cdc-wdm1; do
        [ -e "$dev" ] && { QMI_DEVICE="$dev"; return 0; }
    done
    return 1
}

# ── Helper: check if modem visible on USB bus ──────────────────
modem_on_usb() {
    # SIMCom SIM7600 shows as 1e0e:9011 or 1e0e:9001 ; Qualcomm 05c6:*
    lsusb 2>/dev/null | grep -iqE '1e0e:|05c6:|simtech|qualcomm|sim7600' && return 0
    return 1
}

# ── Helper: find the modem's USB device path for rebind ────────
find_usb_path() {
    # Returns something like 1-1.3:1.4
    for p in /sys/bus/usb/drivers/qmi_wwan/*/net/"$IFACE"; do
        [ -d "$p" ] && { echo "$p" | sed 's|/sys/bus/usb/drivers/qmi_wwan/||; s|/net/.*||'; return 0; }
    done
    # Fallback: search by vid/pid
    for d in /sys/bus/usb/devices/*/idVendor; do
        dir=$(dirname "$d")
        vid=$(cat "$d" 2>/dev/null)
        pid=$(cat "$dir/idProduct" 2>/dev/null)
        if [[ "$vid" == "1e0e" ]] || [[ "$vid" == "05c6" ]]; then
            for intf in "$dir"/*/driver; do
                intf_dir=$(dirname "$intf")
                basename "$intf_dir" 2>/dev/null
                return 0
            done
        fi
    done
    return 1
}

# ── Step 1: Ensure kernel modules are loaded ───────────────────
load_modules() {
    for mod in qmi_wwan cdc_wdm option; do
        if ! lsmod | grep -q "^$mod"; then
            log "Loading kernel module: $mod"
            modprobe "$mod" 2>/dev/null || log "WARN: modprobe $mod failed (may be built-in)"
        fi
    done
}

# ── Step 2: USB recovery — rebind if wwan0 missing ────────────
usb_recovery() {
    # Already have a QMI device? skip
    find_qmi_dev && return 0

    if ! modem_on_usb; then
        log "ERROR: Modem not visible on USB bus at all"
        return 1
    fi

    log "Modem on USB but no QMI device — attempting USB recovery..."

    # Try 1: reload qmi_wwan
    rmmod qmi_wwan 2>/dev/null || true
    sleep 1
    modprobe qmi_wwan 2>/dev/null || true
    sleep 3
    find_qmi_dev && { log "USB recovery: module reload worked"; return 0; }

    # Try 2: USB rebind — find modem's USB path and rebind driver
    local usb_path
    usb_path=$(find_usb_path 2>/dev/null) || true
    if [ -n "$usb_path" ]; then
        log "USB recovery: rebinding $usb_path..."
        echo "$usb_path" > /sys/bus/usb/drivers/qmi_wwan/unbind 2>/dev/null || true
        sleep 2
        echo "$usb_path" > /sys/bus/usb/drivers/qmi_wwan/bind 2>/dev/null || true
        sleep 3
        find_qmi_dev && { log "USB recovery: rebind worked"; return 0; }
    fi

    # Try 3: full USB port reset
    local usb_dev_dir
    for d in /sys/bus/usb/devices/*/idVendor; do
        local vid pid
        vid=$(cat "$d" 2>/dev/null)
        if [[ "$vid" == "1e0e" ]] || [[ "$vid" == "05c6" ]]; then
            usb_dev_dir=$(dirname "$d")
            if [ -f "$usb_dev_dir/authorized" ]; then
                log "USB recovery: resetting USB port $(basename "$usb_dev_dir")..."
                echo 0 > "$usb_dev_dir/authorized"
                sleep 3
                echo 1 > "$usb_dev_dir/authorized"
                sleep 5
                find_qmi_dev && { log "USB recovery: port reset worked"; return 0; }
            fi
            break
        fi
    done

    # Try 4: AT command to switch USB mode (if ttyUSB exists)
    for port in /dev/ttyUSB2 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB3; do
        if [ -e "$port" ]; then
            log "USB recovery: sending AT+CUSBPIDSWITCH via $port..."
            stty -F "$port" 115200 raw -echo 2>/dev/null || continue
            echo -e "AT+CUSBPIDSWITCH=9011,1,1\r" > "$port" 2>/dev/null || continue
            log "USB mode switch sent — modem will reboot in ~10s"
            sleep 15
            find_qmi_dev && { log "USB recovery: mode switch worked"; return 0; }
            break
        fi
    done

    log "ERROR: All USB recovery methods failed"
    return 1
}

# ── Step 3: Establish QMI data session ─────────────────────────
connect() {
    log "Connecting (APN=$APN, dev=$QMI_DEVICE)..."

    # Kill PPP if running (conflicts with QMI)
    poff -a 2>/dev/null || true
    killall pppd 2>/dev/null || true

    # Bring interface down first (required for raw_ip switch)
    ip link set "$IFACE" down 2>/dev/null || true

    # Set raw-ip mode
    if [ -e "/sys/class/net/$IFACE/qmi/raw_ip" ]; then
        echo Y > "/sys/class/net/$IFACE/qmi/raw_ip" 2>/dev/null || true
    fi

    ip link set "$IFACE" up 2>/dev/null
    if ! ip link show "$IFACE" 2>/dev/null | grep -q "UP"; then
        log "ERROR: Could not bring $IFACE UP"
        return 1
    fi

    # Stop stale data session
    qmicli -d "$QMI_DEVICE" --wds-stop-network=disable-autoconnect \
        --device-open-proxy 2>/dev/null || true
    sleep 1

    # Start data session
    if ! qmicli -d "$QMI_DEVICE" \
        --wds-start-network="apn=$APN,ip-type=4" \
        --client-no-release-cid \
        --device-open-proxy 2>/dev/null; then
        log "ERROR: wds-start-network failed"
        return 1
    fi

    sleep 2

    # ── DHCP ───────────────────────────────────────────────────
    # Flush old addresses first
    ip addr flush dev "$IFACE" 2>/dev/null || true
    ip link set "$IFACE" up 2>/dev/null || true

    local dhcp_ok=false
    if command -v udhcpc &>/dev/null; then
        # -f = foreground, -q = quit after lease, -n = fail if no lease
        if udhcpc -i "$IFACE" -f -q -n -t 10 -A 5 2>/dev/null; then
            dhcp_ok=true
        fi
    fi
    if ! $dhcp_ok && command -v dhclient &>/dev/null; then
        dhclient -1 "$IFACE" 2>/dev/null && dhcp_ok=true
    fi

    # Verify IP assigned
    sleep 1
    if ! ip addr show "$IFACE" 2>/dev/null | grep -q "inet "; then
        log "ERROR: No IP address after DHCP"
        return 1
    fi

    local ip_addr
    ip_addr=$(ip -4 addr show "$IFACE" | grep inet | awk '{print $2}' | head -1)
    log "IP acquired: $ip_addr"

    # ── DNS hardening ──────────────────────────────────────────
    # This is the critical fix for "can ping but no internet"
    harden_dns

    # ── Route management ───────────────────────────────────────
    # Add wwan0 default route with a reasonable metric so it doesn't
    # fight with WiFi (metric 0) but is available as primary cellular
    local gw
    gw=$(ip route show dev "$IFACE" 2>/dev/null | grep default | awk '{print $3}' | head -1)
    if [ -n "$gw" ]; then
        # Remove any existing wwan0 default route and re-add with metric
        ip route del default dev "$IFACE" 2>/dev/null || true
        ip route add default via "$gw" dev "$IFACE" metric 200 2>/dev/null || true
        log "Default route via $gw metric 200"
    else
        # No gateway from DHCP — add a scope-link route
        ip route add default dev "$IFACE" metric 200 2>/dev/null || true
        log "Default route (link-scope) metric 200"
    fi

    return 0
}

# ── DNS hardening ──────────────────────────────────────────────
harden_dns() {
    # Many QMI/cellular connections don't configure DNS properly.
    # We force well-known public resolvers into resolv.conf, being
    # careful not to blow away existing entries.

    local resolv="/etc/resolv.conf"
    local resolv_bak="/etc/resolv.conf.qmi-backup"

    # Backup original once
    [ ! -f "$resolv_bak" ] && cp "$resolv" "$resolv_bak" 2>/dev/null || true

    # If resolv.conf is a symlink to systemd-resolved, use resolvectl
    if command -v resolvectl &>/dev/null && [ -L "$resolv" ]; then
        resolvectl dns "$IFACE" 8.8.8.8 1.1.1.1 2>/dev/null || true
        resolvectl domain "$IFACE" "~." 2>/dev/null || true
        log "DNS set via resolvectl"
        return
    fi

    # Direct resolv.conf manipulation
    # Add our nameservers at the TOP if not already present
    local tmp
    tmp=$(mktemp)
    {
        for ns in $DNS_SERVERS; do
            echo "nameserver $ns"
        done
        # Keep existing entries that aren't ours
        grep -v "^#.*qmi-fortified" "$resolv" 2>/dev/null | \
            grep -vF "nameserver 8.8.8.8" | \
            grep -vF "nameserver 1.1.1.1" | \
            grep -vF "nameserver 8.8.4.4" || true
    } > "$tmp"
    cp "$tmp" "$resolv"
    rm -f "$tmp"
    log "DNS hardened: $DNS_SERVERS"
}

# ── Main logic ─────────────────────────────────────────────────
main() {
    log "============ QMI connect starting ============"

    load_modules

    # Wait for QMI device (USB enumeration can lag 10-30s after boot)
    log "Searching for QMI device..."
    local found=false
    for i in $(seq 1 15); do
        if find_qmi_dev; then
            found=true
            break
        fi
        [ "$i" -le 3 ] && sleep 2 || sleep 4
    done

    # If not found, try USB recovery
    if ! $found; then
        usb_recovery || { log "FATAL: Cannot find QMI device"; exit 1; }
    fi
    log "Using QMI device: $QMI_DEVICE"

    # Connection attempts with exponential back-off
    for attempt in $(seq 1 $MAX_RETRIES); do
        if connect; then
            log "✓ QMI connection established"
            exit 0
        fi
        local backoff=$(( attempt * 5 ))
        log "Attempt $attempt/$MAX_RETRIES failed — retrying in ${backoff}s..."
        sleep "$backoff"
    done

    log "FATAL: Failed after $MAX_RETRIES attempts"
    exit 1
}

main "$@"
CONNECTEOF

# Substitute APN
sed -i "s|__APN__|$APN|g" /usr/local/bin/qmi-connect.sh
chmod +x /usr/local/bin/qmi-connect.sh

# ===================================================================
#  Deploy /usr/local/bin/qmi-watchdog.sh  (fortified watchdog)
# ===================================================================
echo "[4/5] Installing watchdog script..."

cat > /usr/local/bin/qmi-watchdog.sh << 'WATCHDOGEOF'
#!/usr/bin/env bash
# ── Fortified QMI watchdog — multi-layer health monitoring ─────
# Runs as a systemd service.  Every 30 s it checks:
#   Layer 1: wwan0 interface exists & is UP
#   Layer 2: wwan0 has an IP address
#   Layer 3: Can ping an external IP  (raw connectivity)
#   Layer 4: DNS resolution works     (name → IP)
#   Layer 5: HTTP reachable           (full stack)
#
# If any layer fails, it escalates recovery:
#   Soft: just fix DNS / bring interface up
#   Medium: full QMI reconnect
#   Hard: USB recovery + reconnect
# ────────────────────────────────────────────────────────────────

IFACE="wwan0"
PING_TARGETS="8.8.8.8 1.1.1.1"
DNS_TEST_HOST="google.com"
HTTP_TEST_URL="http://clients3.google.com/generate_204"
CHECK_INTERVAL=30
DNS_SERVERS="8.8.8.8 1.1.1.1 8.8.4.4"

# Escalation tracking
consecutive_failures=0
SOFT_THRESHOLD=1       # fix DNS / bring iface up
MEDIUM_THRESHOLD=2     # full QMI reconnect
HARD_THRESHOLD=4       # USB recovery + reconnect

log() { echo "$(date '+%Y-%m-%d %H:%M:%S') [qmi-watchdog] $*" | tee -a /var/log/qmi-watchdog.log; }

# ── Layer checks ───────────────────────────────────────────────

check_interface_up() {
    ip link show "$IFACE" 2>/dev/null | grep -q "state UP" && return 0
    ip link show "$IFACE" 2>/dev/null | grep -q "LOWER_UP" && return 0
    return 1
}

check_ip_assigned() {
    ip addr show "$IFACE" 2>/dev/null | grep -q "inet " && return 0
    return 1
}

check_ping() {
    for target in $PING_TARGETS; do
        if ping -I "$IFACE" -c 1 -W 4 "$target" &>/dev/null; then
            return 0
        fi
    done
    return 1
}

check_dns() {
    # Try nslookup / host / dig — whichever is available
    if command -v nslookup &>/dev/null; then
        nslookup "$DNS_TEST_HOST" 8.8.8.8 </dev/null &>/dev/null && return 0
    fi
    if command -v host &>/dev/null; then
        host -W 4 "$DNS_TEST_HOST" 8.8.8.8 &>/dev/null && return 0
    fi
    if command -v dig &>/dev/null; then
        dig +short +time=4 "$DNS_TEST_HOST" @8.8.8.8 &>/dev/null && return 0
    fi
    # Fallback: try ping by hostname (uses system resolver)
    ping -c 1 -W 4 "$DNS_TEST_HOST" &>/dev/null && return 0
    return 1
}

check_http() {
    # Google's generate_204 endpoint — lightweight, returns 204 if internet works
    if command -v curl &>/dev/null; then
        local code
        code=$(curl -s -o /dev/null -w "%{http_code}" --interface "$IFACE" \
               --connect-timeout 5 --max-time 8 "$HTTP_TEST_URL" 2>/dev/null)
        [[ "$code" == "204" || "$code" == "200" ]] && return 0
    elif command -v wget &>/dev/null; then
        wget -q --bind-address="$(ip -4 addr show "$IFACE" | grep inet | awk '{print $2}' | cut -d/ -f1 | head -1)" \
             --timeout=8 --spider "$HTTP_TEST_URL" 2>/dev/null && return 0
    fi
    return 1
}

# ── Recovery actions ───────────────────────────────────────────

soft_recover() {
    log "SOFT recovery: fixing DNS and interface..."

    # Bring interface up if down
    ip link set "$IFACE" up 2>/dev/null || true

    # Harden DNS
    local resolv="/etc/resolv.conf"
    local tmp
    tmp=$(mktemp)
    {
        for ns in $DNS_SERVERS; do
            echo "nameserver $ns"
        done
        grep -vF "nameserver 8.8.8.8" "$resolv" 2>/dev/null | \
            grep -vF "nameserver 1.1.1.1" | \
            grep -vF "nameserver 8.8.4.4" || true
    } > "$tmp"
    cp "$tmp" "$resolv"
    rm -f "$tmp"

    # Ensure default route exists for wwan0
    if ! ip route show default dev "$IFACE" &>/dev/null; then
        ip route add default dev "$IFACE" metric 200 2>/dev/null || true
    fi
}

medium_recover() {
    log "MEDIUM recovery: full QMI reconnect..."
    /usr/local/bin/qmi-connect.sh 2>&1 | tail -20
}

hard_recover() {
    log "HARD recovery: USB reset + QMI reconnect..."

    # Try to power-cycle the USB modem
    for d in /sys/bus/usb/devices/*/idVendor; do
        local vid
        vid=$(cat "$d" 2>/dev/null)
        if [[ "$vid" == "1e0e" ]] || [[ "$vid" == "05c6" ]]; then
            local usb_dir
            usb_dir=$(dirname "$d")
            if [ -f "$usb_dir/authorized" ]; then
                log "HARD: resetting USB device $(basename "$usb_dir")..."
                echo 0 > "$usb_dir/authorized" 2>/dev/null || true
                sleep 5
                echo 1 > "$usb_dir/authorized" 2>/dev/null || true
                sleep 10
            fi
            break
        fi
    done

    # Now try connecting
    /usr/local/bin/qmi-connect.sh 2>&1 | tail -20
}

# ── Main watchdog loop ─────────────────────────────────────────

log "========= QMI watchdog starting ========="
log "Check interval: ${CHECK_INTERVAL}s"

while true; do
    sleep "$CHECK_INTERVAL"

    # ── Layer 1: Interface exists and is UP ────────────────────
    if ! check_interface_up; then
        log "FAIL layer 1: $IFACE not UP"
        consecutive_failures=$((consecutive_failures + 1))

        if [ "$consecutive_failures" -ge "$HARD_THRESHOLD" ]; then
            hard_recover
        elif [ "$consecutive_failures" -ge "$MEDIUM_THRESHOLD" ]; then
            medium_recover
        else
            soft_recover
        fi
        continue
    fi

    # ── Layer 2: IP address assigned ───────────────────────────
    if ! check_ip_assigned; then
        log "FAIL layer 2: No IP on $IFACE"
        consecutive_failures=$((consecutive_failures + 1))

        if [ "$consecutive_failures" -ge "$MEDIUM_THRESHOLD" ]; then
            medium_recover
        else
            # Just try DHCP again
            log "Retrying DHCP..."
            if command -v udhcpc &>/dev/null; then
                udhcpc -i "$IFACE" -f -q -n -t 8 2>/dev/null || true
            elif command -v dhclient &>/dev/null; then
                dhclient -1 "$IFACE" 2>/dev/null || true
            fi
            soft_recover
        fi
        continue
    fi

    # ── Layer 3: Ping ──────────────────────────────────────────
    if ! check_ping; then
        log "FAIL layer 3: Ping failed"
        consecutive_failures=$((consecutive_failures + 1))

        if [ "$consecutive_failures" -ge "$HARD_THRESHOLD" ]; then
            hard_recover
        elif [ "$consecutive_failures" -ge "$MEDIUM_THRESHOLD" ]; then
            medium_recover
        else
            soft_recover
        fi
        continue
    fi

    # ── Layer 4: DNS ───────────────────────────────────────────
    if ! check_dns; then
        log "FAIL layer 4: DNS resolution failed (ping OK!)"
        consecutive_failures=$((consecutive_failures + 1))
        # DNS failures are always soft-recoverable
        soft_recover
        continue
    fi

    # ── Layer 5: HTTP (optional — only if curl available) ──────
    if command -v curl &>/dev/null; then
        if ! check_http; then
            log "WARN layer 5: HTTP check failed (ping+DNS OK)"
            # Don't escalate hard for HTTP — might be transient
            consecutive_failures=$((consecutive_failures + 1))
            if [ "$consecutive_failures" -ge "$MEDIUM_THRESHOLD" ]; then
                soft_recover
            fi
            continue
        fi
    fi

    # ── All checks passed ──────────────────────────────────────
    if [ "$consecutive_failures" -gt 0 ]; then
        log "✓ All layers healthy — recovered after $consecutive_failures failures"
    fi
    consecutive_failures=0
done
WATCHDOGEOF

chmod +x /usr/local/bin/qmi-watchdog.sh

# ===================================================================
#  Deploy systemd services
# ===================================================================
echo "[5/5] Installing systemd services..."

# ── qmi-network.service (connection on boot) ──────────────────
cat > /etc/systemd/system/qmi-network.service << 'SVCEOF'
[Unit]
Description=Fortified QMI Network (SIM7600E)
After=network-pre.target systemd-modules-load.service
Wants=network-pre.target
Before=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStartPre=/sbin/modprobe qmi_wwan
ExecStartPre=/sbin/modprobe cdc_wdm
ExecStart=/usr/local/bin/qmi-connect.sh
ExecStop=/usr/bin/qmicli -d /dev/cdc-wdm0 --wds-stop-network=disable-autoconnect --device-open-proxy
Restart=on-failure
RestartSec=15
TimeoutStartSec=120

[Install]
WantedBy=multi-user.target
SVCEOF

# ── qmi-watchdog.service (resilient monitoring) ───────────────
cat > /etc/systemd/system/qmi-watchdog.service << 'WDEOF'
[Unit]
Description=Fortified QMI Connection Watchdog
After=qmi-network.service
BindsTo=qmi-network.service

[Service]
Type=simple
ExecStart=/usr/local/bin/qmi-watchdog.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
WDEOF

# ── Disable old PPP / old QMI services if they exist ──────────
systemctl disable ppp@sim7600e 2>/dev/null || true
systemctl stop ppp@sim7600e 2>/dev/null || true

# ── Reload and enable ─────────────────────────────────────────
systemctl daemon-reload
systemctl enable qmi-network.service
systemctl enable qmi-watchdog.service

# ── Log rotation (prevent /var/log/qmi-*.log from growing) ────
cat > /etc/logrotate.d/qmi-fortified << 'LOGEOF'
/var/log/qmi-connect.log /var/log/qmi-watchdog.log {
    daily
    rotate 7
    compress
    missingok
    notifempty
    size 5M
}
LOGEOF

# ── Verify boot enablement ────────────────────────────────────
echo ""
echo "Verifying boot configuration..."
for svc in qmi-network qmi-watchdog; do
    if systemctl is-enabled "$svc" &>/dev/null; then
        echo "  ✓ $svc is ENABLED (will start on boot)"
    else
        echo "  ✗ $svc is NOT enabled — forcing enable..."
        systemctl enable "$svc" 2>/dev/null || true
    fi
done

# ── Start immediately so user doesn't have to reboot ──────────
echo ""
echo "Starting services now..."
systemctl start qmi-network 2>/dev/null && echo "  ✓ qmi-network started" || echo "  ⚠ qmi-network start failed (will retry on boot)"
sleep 2
systemctl start qmi-watchdog 2>/dev/null && echo "  ✓ qmi-watchdog started" || echo "  ⚠ qmi-watchdog start failed (will retry on boot)"

# ── If robotcontrol service exists, update its dependencies ───
if [ -f /etc/systemd/system/robotcontrol.service ]; then
    echo ""
    echo "Updating robotcontrol.service to depend on QMI network..."
    if ! grep -q 'qmi-network.service' /etc/systemd/system/robotcontrol.service; then
        sed -i 's|^After=\(.*\)|After=\1 qmi-network.service|' /etc/systemd/system/robotcontrol.service
        if ! grep -q '^Wants=.*qmi-network' /etc/systemd/system/robotcontrol.service; then
            sed -i '/^After=/a Wants=qmi-network.service' /etc/systemd/system/robotcontrol.service
        fi
        systemctl daemon-reload
        echo "  ✓ robotcontrol.service updated — will wait for QMI on boot"
    else
        echo "  ✓ robotcontrol.service already depends on qmi-network"
    fi
fi

echo ""
echo "==========================================="
echo " Fortified QMI Auto-Connect Installed"
echo "==========================================="
echo ""
echo "  ✓ BOOT AUTO-START: ENABLED"
echo ""
echo "Services (auto-start on boot):"
echo "  qmi-network.service   — connects on boot (with USB recovery)"
echo "  qmi-watchdog.service  — 30s health check with escalating recovery"
echo ""
echo "Boot order:  kernel modules → qmi-network → qmi-watchdog → robotcontrol"
echo ""
echo "Health checks (every 30s):"
echo "  Layer 1: Interface UP"
echo "  Layer 2: IP assigned"
echo "  Layer 3: Ping 8.8.8.8 / 1.1.1.1"
echo "  Layer 4: DNS resolution"
echo "  Layer 5: HTTP connectivity"
echo ""
echo "Logs:"
echo "  journalctl -u qmi-network -f"
echo "  journalctl -u qmi-watchdog -f"
echo "  tail -f /var/log/qmi-connect.log"
echo "  tail -f /var/log/qmi-watchdog.log"
echo ""
echo "Manage:"
echo "  sudo systemctl status qmi-network   — check connection"
echo "  sudo systemctl restart qmi-network  — force reconnect"
echo "  sudo systemctl stop qmi-network     — disconnect"
echo ""
