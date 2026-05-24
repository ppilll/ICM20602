#!/bin/sh
# ICM20602 FIFO low-frequency validation script
#
# Purpose:
#   1. One-terminal test: background dmesg capture.
#   2. Focus on stable 50Hz validation first.
#   3. Verify:
#        - data-ready path can read frames
#        - FIFO watermark path can read frames
#        - FIFO watermark interrupt count roughly matches frames / watermark
#        - fifo_stats is read if the driver exposes it
#   4. No python3 dependency on the board.
#
# Usage:
#   chmod +x test_icm20602_fifo_50hz.sh
#   ./test_icm20602_fifo_50hz.sh
#
# Optional:
#   DEV=/sys/bus/iio/devices/iio:device0 ./test_icm20602_fifo_50hz.sh
#   OUTDIR=/tmp/icm20602_test_50hz ./test_icm20602_fifo_50hz.sh
#   DRDY_HZ=50 FIFO_HZ=50 FIFO_WM=8 FIFO_FRAMES=200 ./test_icm20602_fifo_50hz.sh
#
# Expected default size:
#   SCAN_BYTES=24
#   DRDY_FRAMES=50    -> 1200 bytes
#   FIFO_FRAMES=200   -> 4800 bytes

set -u

OUTDIR="${OUTDIR:-/tmp/icm20602_test_50hz}"
SCAN_BYTES="${SCAN_BYTES:-24}"

DRDY_HZ="${DRDY_HZ:-50}"
DRDY_FRAMES="${DRDY_FRAMES:-50}"

FIFO_HZ="${FIFO_HZ:-50}"
FIFO_WM="${FIFO_WM:-8}"
FIFO_FRAMES="${FIFO_FRAMES:-200}"

BUFFER_LENGTH="${BUFFER_LENGTH:-1024}"
BUFFER_WM_DRDY="${BUFFER_WM_DRDY:-1}"
BUFFER_WM_FIFO="${BUFFER_WM_FIFO:-8}"

mkdir -p "$OUTDIR"

log()
{
    echo "[$(date '+%H:%M:%S')] $*"
}

cleanup()
{
    if [ -n "${DMESG_PID:-}" ]; then
        kill "$DMESG_PID" 2>/dev/null || true
        wait "$DMESG_PID" 2>/dev/null || true
    fi

    if [ -n "${DEV:-}" ] && [ -d "$DEV" ]; then
        echo 0 > "$DEV/buffer/enable" 2>/dev/null || true
    fi
}

die()
{
    echo "ERROR: $*" >&2
    cleanup
    exit 1
}

trap cleanup EXIT INT TERM

find_iio_dev()
{
    if [ -n "${DEV:-}" ]; then
        [ -d "$DEV" ] || die "DEV does not exist: $DEV"
        return
    fi

    for d in /sys/bus/iio/devices/iio:device*; do
        [ -d "$d" ] || continue
        name="$(cat "$d/name" 2>/dev/null || true)"
        if echo "$name" | grep -qi "icm20602"; then
            DEV="$d"
            return
        fi
    done

    die "cannot find IIO device named icm20602"
}

find_chr_dev()
{
    idx="${DEV##*iio:device}"
    CHR="/dev/iio:device$idx"
    [ -e "$CHR" ] || die "cannot find character device $CHR"
}

find_triggers()
{
    DRDY_TRIG=""
    FIFO_TRIG=""

    for t in /sys/bus/iio/devices/trigger*; do
        [ -e "$t/name" ] || continue
        n="$(cat "$t/name" 2>/dev/null || true)"

        if [ -z "$DRDY_TRIG" ] && echo "$n" | grep -qi "drdy"; then
            DRDY_TRIG="$n"
        fi

        if [ -z "$FIFO_TRIG" ] && echo "$n" | grep -qi "fifo"; then
            FIFO_TRIG="$n"
        fi
    done

    log "DRDY_TRIG=$DRDY_TRIG"
    log "FIFO_TRIG=$FIFO_TRIG"
}

mount_debugfs()
{
    mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
}

enable_dynamic_debug()
{
    # This is intentionally simple. Your previous script failed because it tried
    # to feed absolute paths with Chinese characters into the dynamic-debug query.
    if [ -e /sys/kernel/debug/dynamic_debug/control ]; then
        echo 'file ICM20602.c +p' > /sys/kernel/debug/dynamic_debug/control 2>/dev/null || true
        echo 'file icm20602.c +p' > /sys/kernel/debug/dynamic_debug/control 2>/dev/null || true
        log "dynamic debug requested for ICM20602.c/icm20602.c"
    else
        log "dynamic debug control not available"
    fi
}

start_dmesg()
{
    dmesg -c > "$OUTDIR/dmesg_before_clear.log" 2>/dev/null || true
    dmesg -w > "$OUTDIR/dmesg_live.log" &
    DMESG_PID=$!
    log "dmesg background pid=$DMESG_PID"
}

stop_buffer()
{
    echo 0 > "$DEV/buffer/enable" 2>/dev/null || true
}

set_attr_if_exists()
{
    path="$1"
    value="$2"

    if [ -e "$path" ]; then
        echo "$value" > "$path" || die "failed to write $value to $path"
    else
        log "skip missing attr: $path"
    fi
}

set_sampling_frequency()
{
    hz="$1"

    if [ -e "$DEV/in_accel_sampling_frequency" ]; then
        echo "$hz" > "$DEV/in_accel_sampling_frequency" || die "failed to set in_accel_sampling_frequency"
    elif [ -e "$DEV/sampling_frequency" ]; then
        echo "$hz" > "$DEV/sampling_frequency" || die "failed to set sampling_frequency"
    elif [ -e "$DEV/in_anglvel_sampling_frequency" ]; then
        echo "$hz" > "$DEV/in_anglvel_sampling_frequency" || die "failed to set in_anglvel_sampling_frequency"
    else
        die "no sampling_frequency sysfs attribute found"
    fi
}

get_sampling_frequency()
{
    cat "$DEV/in_accel_sampling_frequency" 2>/dev/null || \
    cat "$DEV/sampling_frequency" 2>/dev/null || \
    cat "$DEV/in_anglvel_sampling_frequency" 2>/dev/null || \
    echo "unknown"
}

enable_scan_elements()
{
    stop_buffer

    set_attr_if_exists "$DEV/scan_elements/in_accel_x_en" 1
    set_attr_if_exists "$DEV/scan_elements/in_accel_y_en" 1
    set_attr_if_exists "$DEV/scan_elements/in_accel_z_en" 1
    set_attr_if_exists "$DEV/scan_elements/in_anglvel_x_en" 1
    set_attr_if_exists "$DEV/scan_elements/in_anglvel_y_en" 1
    set_attr_if_exists "$DEV/scan_elements/in_anglvel_z_en" 1

    if [ -e "$DEV/scan_elements/in_timestamp_en" ]; then
        echo 1 > "$DEV/scan_elements/in_timestamp_en" || true
    fi

    {
        echo "scan elements:"
        for f in "$DEV"/scan_elements/*_en "$DEV"/scan_elements/*_type "$DEV"/scan_elements/*_index; do
            [ -e "$f" ] && echo "$(basename "$f")=$(cat "$f" 2>/dev/null)"
        done
    } | tee "$OUTDIR/scan_elements_dump.txt"
}

show_inventory()
{
    {
        echo "IIO devices:"
        for d in /sys/bus/iio/devices/iio:device*; do
            [ -d "$d" ] || continue
            echo "  $d: $(cat "$d/name" 2>/dev/null || echo '?')"
        done

        echo "Triggers:"
        for t in /sys/bus/iio/devices/trigger*; do
            [ -e "$t/name" ] || continue
            echo "  $t: $(cat "$t/name" 2>/dev/null || echo '?')"
        done

        echo "Using DEV=$DEV"
        echo "Using CHR=$CHR"
    } | tee "$OUTDIR/inventory.txt"
}

get_irq_count_for_name()
{
    name="$1"
    awk -v n="$name" '$0 ~ n {sum += $2} END {print sum+0}' /proc/interrupts 2>/dev/null
}

dump_stats()
{
    tag="$1"
    out="$OUTDIR/stats_${tag}.txt"

    {
        echo "tag=$tag"
        echo "time=$(date '+%F %T')"
        echo "DEV=$DEV"
        echo "CHR=$CHR"
        echo "current_trigger=$(cat "$DEV/trigger/current_trigger" 2>/dev/null || true)"
        echo "buffer_length=$(cat "$DEV/buffer/length" 2>/dev/null || true)"
        echo "buffer_watermark=$(cat "$DEV/buffer/watermark" 2>/dev/null || true)"
        echo "fifo_watermark=$(cat "$DEV/fifo_watermark" 2>/dev/null || true)"
        echo "sampling_frequency=$(get_sampling_frequency)"
        echo "--- fifo_stats ---"
        if [ -e "$DEV/fifo_stats" ]; then
            cat "$DEV/fifo_stats"
        else
            echo "fifo_stats not available"
            echo "NOTE: fifo_stats must be implemented in the driver, not in this script."
        fi
        echo "--- interrupts ---"
        grep -E "spi0.0|icm20602|fifo|drdy|ecspi|gpio" /proc/interrupts 2>/dev/null || true
    } | tee "$out"
}

capture_dd()
{
    tag="$1"
    count="$2"
    outfile="$OUTDIR/${tag}.bin"

    rm -f "$outfile"

    log "capture $tag: dd bs=$SCAN_BYTES count=$count"
    dd if="$CHR" of="$outfile" bs="$SCAN_BYTES" count="$count" iflag=fullblock 2>"$OUTDIR/dd_${tag}.log"

    bytes="$(wc -c < "$outfile" 2>/dev/null || echo 0)"
    frames=$((bytes / SCAN_BYTES))
    expected=$((count * SCAN_BYTES))

    log "$tag result: bytes=$bytes frames=$frames expected_bytes=$expected"
}

summarize_file()
{
    tag="$1"
    file="$OUTDIR/${tag}.bin"

    if [ ! -s "$file" ]; then
        echo "$tag: empty or missing"
        return
    fi

    bytes="$(wc -c < "$file")"
    frames=$((bytes / SCAN_BYTES))
    echo "$tag: bytes=$bytes frames=$frames"
}

run_drdy_50hz()
{
    if [ -z "$DRDY_TRIG" ]; then
        log "skip DRDY test: DRDY trigger not found"
        return
    fi

    log "=== TEST A: data-ready ${DRDY_HZ}Hz sanity ==="

    stop_buffer
    echo "$DRDY_TRIG" > "$DEV/trigger/current_trigger" || die "failed to set DRDY trigger"

    set_sampling_frequency "$DRDY_HZ"
    set_attr_if_exists "$DEV/buffer/length" "$BUFFER_LENGTH"
    set_attr_if_exists "$DEV/buffer/watermark" "$BUFFER_WM_DRDY"

    dump_stats "drdy_before"

    irq_before="$(get_irq_count_for_name "$DRDY_TRIG")"

    echo 1 > "$DEV/buffer/enable" || die "failed to enable buffer for DRDY"
    capture_dd "drdy_${DRDY_HZ}hz_${DRDY_FRAMES}f" "$DRDY_FRAMES"
    echo 0 > "$DEV/buffer/enable" || true

    irq_after="$(get_irq_count_for_name "$DRDY_TRIG")"
    irq_delta=$((irq_after - irq_before))

    dump_stats "drdy_after"

    {
        echo "DRDY_HZ=$DRDY_HZ"
        echo "DRDY_FRAMES=$DRDY_FRAMES"
        echo "DRDY_IRQ_DELTA=$irq_delta"
        echo "Expected roughly: close to DRDY_FRAMES, because data-ready triggers once per frame."
    } | tee "$OUTDIR/result_drdy_irq.txt"
}

run_fifo_50hz()
{
    if [ -z "$FIFO_TRIG" ]; then
        log "skip FIFO test: FIFO trigger not found"
        return
    fi

    log "=== TEST B: FIFO watermark ${FIFO_HZ}Hz wm=${FIFO_WM} ==="

    stop_buffer
    echo "$FIFO_TRIG" > "$DEV/trigger/current_trigger" || die "failed to set FIFO trigger"

    set_sampling_frequency "$FIFO_HZ"
    set_attr_if_exists "$DEV/fifo_watermark" "$FIFO_WM"
    set_attr_if_exists "$DEV/buffer/length" "$BUFFER_LENGTH"
    set_attr_if_exists "$DEV/buffer/watermark" "$BUFFER_WM_FIFO"

    dump_stats "fifo_before"

    irq_before="$(get_irq_count_for_name "$FIFO_TRIG")"

    echo 1 > "$DEV/buffer/enable" || die "failed to enable buffer for FIFO"
    capture_dd "fifo_${FIFO_HZ}hz_wm${FIFO_WM}_${FIFO_FRAMES}f" "$FIFO_FRAMES"
    echo 0 > "$DEV/buffer/enable" || true

    irq_after="$(get_irq_count_for_name "$FIFO_TRIG")"
    irq_delta=$((irq_after - irq_before))

    dump_stats "fifo_after"

    expected_irq=$(( (FIFO_FRAMES + FIFO_WM - 1) / FIFO_WM ))

    {
        echo "FIFO_HZ=$FIFO_HZ"
        echo "FIFO_WM=$FIFO_WM"
        echo "FIFO_FRAMES=$FIFO_FRAMES"
        echo "FIFO_IRQ_DELTA=$irq_delta"
        echo "Expected roughly: FIFO_FRAMES / FIFO_WM = $FIFO_FRAMES / $FIFO_WM ~= $expected_irq"
        echo
        if [ -e "$DEV/fifo_stats" ]; then
            echo "fifo_stats after:"
            cat "$DEV/fifo_stats"
            echo
            echo "If fifo_frames_pushed and fifo_burst_read_count exist:"
            echo "  avg_frames_per_burst = fifo_frames_pushed / fifo_burst_read_count"
            echo "  For wm=$FIFO_WM, avg should be close to $FIFO_WM in stable capture."
        else
            echo "fifo_stats not available, so burst-read average cannot be computed by script."
        fi
    } | tee "$OUTDIR/result_fifo_irq_and_burst.txt"
}

main()
{
    find_iio_dev
    find_chr_dev
    mount_debugfs
    start_dmesg
    enable_dynamic_debug
    show_inventory
    find_triggers
    enable_scan_elements

    run_drdy_50hz
    run_fifo_50hz

    log "=== SUMMARY ==="
    {
        echo "OUTDIR=$OUTDIR"
        echo
        summarize_file "drdy_${DRDY_HZ}hz_${DRDY_FRAMES}f"
        summarize_file "fifo_${FIFO_HZ}hz_wm${FIFO_WM}_${FIFO_FRAMES}f"
        echo
        echo "Expected bytes:"
        echo "  DRDY: $((DRDY_FRAMES * SCAN_BYTES))"
        echo "  FIFO: $((FIFO_FRAMES * SCAN_BYTES))"
        echo
        echo "IRQ summary:"
        cat "$OUTDIR/result_drdy_irq.txt" 2>/dev/null || true
        echo
        cat "$OUTDIR/result_fifo_irq_and_burst.txt" 2>/dev/null || true
        echo
        echo "Recent dmesg:"
        tail -n 80 "$OUTDIR/dmesg_live.log" 2>/dev/null || true
    } | tee "$OUTDIR/summary.txt"

    log "done. Results saved in $OUTDIR"
}

main "$@"
