#!/usr/bin/env bash
# test_tare.sh — automated tare test for wheeltec_imu kernel driver
#
# Usage:
#   sudo ./test_tare.sh [gyro|acce|level|all]   (default: all)
#
# Requires: driver already loaded, /dev/wheeltec_imu0_* present.
# Exit code: 0 = all tests passed, 1 = one or more failed.

set -euo pipefail

CTRL=/dev/wheeltec_imu0_ctrl
STATE=/dev/wheeltec_imu0_state
RAW=/dev/wheeltec_imu0_raw

TARE_WAIT_MS=2500          # calibration dwell time passed to driver
POLL_INTERVAL=1            # seconds between state polls
TARE_TIMEOUT=60            # max seconds to wait for tare to finish
STREAM_CHECK_SECS=5        # seconds to sample raw stream after tare
STREAM_MIN_BYTES=100       # minimum bytes expected in stream check

PASS=0
FAIL=0
SKIP=0

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "  $*"; }
pass() { echo -e "  ${GREEN}[PASS]${NC} $*"; ((PASS++)); }
fail() { echo -e "  ${RED}[FAIL]${NC} $*"; ((FAIL++)); }
skip() { echo -e "  ${YELLOW}[SKIP]${NC} $*"; ((SKIP++)); }
header() { echo -e "\n${YELLOW}=== $* ===${NC}"; }

# ── Pre-flight checks ────────────────────────────────────────────────────────

preflight() {
    header "Pre-flight checks"

    if [[ $EUID -ne 0 ]]; then
        fail "Must run as root (sudo)"
        exit 1
    fi

    for dev in "$CTRL" "$STATE" "$RAW"; do
        if [[ -e "$dev" ]]; then
            pass "$dev exists"
        else
            fail "$dev not found — is the driver loaded?"
            exit 1
        fi
    done

    local state
    state=$(cat "$STATE")
    if echo "$state" | grep -q "connected=1"; then
        pass "IMU connected"
    else
        fail "IMU not connected (state: $state)"
        exit 1
    fi

    local frames_before
    frames_before=$(echo "$state" | grep raw_frames_ok | cut -d= -f2)
    log "raw_frames_ok before: $frames_before"
    if [[ "$frames_before" -gt 0 ]]; then
        pass "IMU streaming (frames_ok=$frames_before)"
    else
        fail "IMU not streaming before test"
        exit 1
    fi
}

# ── Helpers ──────────────────────────────────────────────────────────────────

read_state_field() {
    # read_state_field <field>  →  prints value
    cat "$STATE" | grep "^${1}=" | cut -d= -f2
}

wait_tare_done() {
    # Returns 0 (success) or 1 (timeout/fail)
    local deadline=$(( $(date +%s) + TARE_TIMEOUT ))
    while [[ $(date +%s) -lt $deadline ]]; do
        local ts
        ts=$(read_state_field tare_status 2>/dev/null || echo "0")
        case "$ts" in
            2) return 0 ;;  # SUCCESS
            3) return 1 ;;  # FAIL/WARNING
            0) # 0 after we started = aborted or done-and-reset
               # give it one more second before treating as abort
               sleep 1
               ts=$(read_state_field tare_status 2>/dev/null || echo "0")
               [[ "$ts" == "2" ]] && return 0
               return 1
               ;;
        esac
        sleep "$POLL_INTERVAL"
    done
    return 1  # timeout
}

check_stream_alive() {
    # Returns 0 if bytes flowing on raw device, 1 otherwise
    local bytes
    bytes=$(dd if="$RAW" bs=1 count=4096 iflag=nonblock 2>/dev/null | wc -c || true)
    # nonblock may return 0 if buffer empty; try a timed read
    if [[ "$bytes" -lt "$STREAM_MIN_BYTES" ]]; then
        bytes=$(timeout "$STREAM_CHECK_SECS" dd if="$RAW" bs=512 count=64 2>/dev/null | wc -c || true)
    fi
    [[ "$bytes" -ge "$STREAM_MIN_BYTES" ]]
}

record_frames() {
    read_state_field raw_frames_ok
}

# ── Single tare test ─────────────────────────────────────────────────────────

run_tare_test() {
    local tare_type=$1   # gyro | acce | level
    header "Tare test: $tare_type"

    # Snapshot frames before
    local frames_before
    frames_before=$(record_frames)
    log "frames_before=$frames_before"

    # Snapshot dmesg position
    local dmesg_pos
    dmesg_pos=$(dmesg | wc -l)

    # Send tare command
    log "Writing: tare:${tare_type}:${TARE_WAIT_MS} → $CTRL"
    echo -n "tare:${tare_type}:${TARE_WAIT_MS}" > "$CTRL"

    # Check tare_pending set immediately
    sleep 0.2
    local tp
    tp=$(read_state_field tare_pending 2>/dev/null || echo "0")
    if [[ "$tp" == "1" ]]; then
        pass "tare_pending=1 after write"
    else
        fail "tare_pending not set (got $tp) — driver may have rejected command"
        return
    fi

    # tare_status should be 1 (in-progress) quickly
    sleep 0.3
    local ts
    ts=$(read_state_field tare_status 2>/dev/null || echo "0")
    if [[ "$ts" == "1" ]]; then
        pass "tare_status=1 (in-progress)"
    else
        log "tare_status=$ts (may have progressed already)"
    fi

    # Wait for tare to complete
    log "Waiting up to ${TARE_TIMEOUT}s for tare to finish..."
    local t_start=$SECONDS
    if wait_tare_done; then
        local elapsed=$(( SECONDS - t_start ))
        pass "tare_status=2 (SUCCESS) after ${elapsed}s"
    else
        local ts_final
        ts_final=$(read_state_field tare_status 2>/dev/null || echo "?")
        fail "tare did not succeed (tare_status=$ts_final after ${TARE_TIMEOUT}s)"
        # Still check stream below
    fi

    # Check stream resumed
    log "Checking data stream after tare..."
    local frames_after
    frames_after=$(record_frames)
    log "frames_after=$frames_after"

    if [[ "$frames_after" -gt "$frames_before" ]]; then
        pass "frames increased (${frames_before} → ${frames_after}) — stream alive"
    else
        # Try live read
        if check_stream_alive; then
            pass "stream alive (live read check)"
        else
            fail "no stream after tare (frames stuck at $frames_after)"
        fi
    fi

    # Check dmesg for expected log sequence
    log "Checking dmesg for tare sequence..."
    local dmesg_new
    dmesg_new=$(dmesg | tail -n +"$dmesg_pos")

    local -a expected_msgs=(
        "TX >> #fconfig"
        "TX >> #fimucal_${tare_type}"
        "TX >> #fsave"
        "TX >> #fconfig (2nd)"
        "TX >> #freboot"
        "TX >> y"
    )
    for msg in "${expected_msgs[@]}"; do
        if echo "$dmesg_new" | grep -q "$msg"; then
            pass "dmesg: '$msg'"
        else
            fail "dmesg: '$msg' not found"
        fi
    done

    # Check for SUCCESS or WARNING in dmesg
    if echo "$dmesg_new" | grep -q "tare: SUCCESS"; then
        pass "dmesg: tare SUCCESS logged"
    elif echo "$dmesg_new" | grep -q "tare: WARNING"; then
        fail "dmesg: tare WARNING — stream did not resume"
    elif echo "$dmesg_new" | grep -q "USB disconnect detected"; then
        pass "dmesg: USB disconnect path (probe restored stream)"
    else
        fail "dmesg: no SUCCESS/WARNING/disconnect message found"
    fi
}

# ── Edge case: duplicate tare rejected ───────────────────────────────────────

test_duplicate_rejected() {
    header "Edge case: duplicate tare while busy"

    # Start a tare
    echo -n "tare:gyro:${TARE_WAIT_MS}" > "$CTRL"
    sleep 0.3

    local tp
    tp=$(read_state_field tare_pending 2>/dev/null || echo "0")
    if [[ "$tp" != "1" ]]; then
        skip "tare not started, skipping duplicate test"
        return
    fi

    # Try to send another tare while first is running
    local out
    out=$(echo -n "tare:gyro:${TARE_WAIT_MS}" > "$CTRL" 2>&1 || true)
    sleep 0.2

    # tare_pending should still be 1 (second command rejected)
    tp=$(read_state_field tare_pending 2>/dev/null || echo "0")
    if [[ "$tp" == "1" ]]; then
        pass "duplicate tare rejected (tare_pending still 1)"
    else
        fail "duplicate tare may have interfered (tare_pending=$tp)"
    fi

    # Wait for first tare to finish before next test
    log "Waiting for first tare to complete..."
    wait_tare_done || true
    sleep 2
}

# ── Invalid tare type rejected ────────────────────────────────────────────────

test_invalid_type() {
    header "Edge case: invalid tare type"

    local tp_before
    tp_before=$(read_state_field tare_pending 2>/dev/null || echo "0")

    echo -n "tare:badtype:2500" > "$CTRL" 2>/dev/null || true
    sleep 0.5

    local tp_after
    tp_after=$(read_state_field tare_pending 2>/dev/null || echo "0")

    if [[ "$tp_after" == "$tp_before" || "$tp_after" == "0" ]]; then
        pass "invalid tare type rejected (tare_pending unchanged)"
    else
        fail "invalid tare type may have been accepted (tare_pending=$tp_after)"
    fi
}

# ── Summary ───────────────────────────────────────────────────────────────────

print_summary() {
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "  ${GREEN}PASS: $PASS${NC}  ${RED}FAIL: $FAIL${NC}  ${YELLOW}SKIP: $SKIP${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    if [[ $FAIL -eq 0 ]]; then
        echo -e "  ${GREEN}All tests passed.${NC}"
    else
        echo -e "  ${RED}$FAIL test(s) failed.${NC}"
    fi
    echo ""
}

# ── Main ──────────────────────────────────────────────────────────────────────

TARGET=${1:-all}

preflight

case "$TARGET" in
    gyro)
        run_tare_test gyro
        ;;
    acce)
        run_tare_test acce
        ;;
    level)
        run_tare_test level
        ;;
    all)
        test_invalid_type

        run_tare_test gyro
        sleep 3

        test_duplicate_rejected
        sleep 3

        run_tare_test acce
        sleep 3

        run_tare_test level
        ;;
    *)
        echo "Usage: $0 [gyro|acce|level|all]"
        exit 1
        ;;
esac

print_summary

[[ $FAIL -eq 0 ]]
