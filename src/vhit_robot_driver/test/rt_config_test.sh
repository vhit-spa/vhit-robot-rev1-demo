#!/usr/bin/env bash
set -u

# poll_wsl_process_config.sh
#
# Usage examples:
#   ./poll_wsl_process_config.sh
#   ./poll_wsl_process_config.sh ros2_control_node 1
#   ./poll_wsl_process_config.sh ros2_control_node 0-1
#   ./poll_wsl_process_config.sh my_test_binary 1
#
# Args:
#   $1 = process pattern to search with pgrep -f
#   $2 = expected CPU affinity list, e.g. 1, 0-1, 2,3
#
# Notes:
#   - Cpus_allowed_list is the important affinity result.
#   - PSR is only the CPU where the thread last ran.
#   - CLS=FF means SCHED_FIFO.
#   - CLS=TS means normal time-sharing scheduling.

PROCESS_PATTERN="${1:-ros2_control_node}"
EXPECTED_CPUS="${2:-1}"
INTERVAL_SEC="${INTERVAL_SEC:-0.10}"

echo "WSL process configuration poller"
echo "Process pattern:        ${PROCESS_PATTERN}"
echo "Expected CPU affinity:  ${EXPECTED_CPUS}"
echo "Polling interval:       ${INTERVAL_SEC}s"
echo

echo "=== WSL / kernel context ==="
uname -a
echo

echo "=== CPUs visible inside WSL ==="
nproc --all 2>/dev/null || true
lscpu 2>/dev/null | grep -E '^CPU\(s\):|^On-line CPU|^Thread|^Core|^Socket' || true
echo

echo "=== Current shell limits ==="
echo "ulimit -r, max realtime priority: $(ulimit -r)"
echo "ulimit -l, max locked memory:     $(ulimit -l)"
echo

echo "Waiting for process matching: ${PROCESS_PATTERN}"
echo

find_pid() {
  pgrep -f "${PROCESS_PATTERN}" | head -n 1
}

print_thread_affinity_from_proc() {
  local pid="$1"

  echo "=== Thread-by-thread /proc status ==="
  for task in /proc/"${pid}"/task/*; do
    [ -d "$task" ] || continue

    local tid
    local comm
    local cpus
    local policy
    local prio

    tid="$(basename "$task")"
    comm="$(cat "$task/comm" 2>/dev/null || true)"
    cpus="$(awk '/Cpus_allowed_list/ {print $2}' "$task/status" 2>/dev/null)"
    policy="$(awk '/^policy/ {print $3}' "$task/sched" 2>/dev/null | head -n 1)"
    prio="$(awk '/^prio/ {print $3}' "$task/sched" 2>/dev/null | head -n 1)"

    printf 'TID=%-8s COMM=%-24s Cpus_allowed_list=%-8s policy=%-4s prio=%s\n' \
      "$tid" "$comm" "${cpus:-?}" "${policy:-?}" "${prio:-?}"
  done
  echo
}

print_report() {
  local pid="$1"

  echo "============================================================"
  date "+%H:%M:%S.%N"
  echo "PID: ${pid}"
  echo

  echo "=== Command ==="
  tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true
  echo
  echo

  echo "=== Process affinity via taskset ==="
  taskset -cp "${pid}" 2>/dev/null || true
  echo

  echo "=== All thread affinities via taskset ==="
  taskset -acp "${pid}" 2>/dev/null || true
  echo

  echo "=== Process /proc allowed CPUs ==="
  grep -E 'Cpus_allowed|Cpus_allowed_list' "/proc/${pid}/status" 2>/dev/null || true
  echo

  echo "=== Threads: scheduling class, RT priority, last CPU ==="
  ps -T -p "${pid}" -o pid,tid,cls,rtprio,pri,psr,comm 2>/dev/null || true
  echo

  print_thread_affinity_from_proc "${pid}"

  local actual_cpus
  actual_cpus="$(awk '/Cpus_allowed_list/ {print $2}' "/proc/${pid}/status" 2>/dev/null)"

  echo "=== Summary ==="

  if [ "${actual_cpus:-}" = "${EXPECTED_CPUS}" ]; then
    echo "Affinity: OK"
    echo "  Cpus_allowed_list=${actual_cpus}"
  else
    echo "Affinity: NOT OK"
    echo "  Cpus_allowed_list=${actual_cpus:-unknown}"
    echo "  Expected=${EXPECTED_CPUS}"
  fi

  if ps -T -p "${pid}" -o cls= 2>/dev/null | grep -q 'FF'; then
    echo "Scheduler: OK, at least one thread is SCHED_FIFO / CLS=FF"
  else
    echo "Scheduler: NOT RT, no thread shows CLS=FF"
  fi

  if ps -T -p "${pid}" -o rtprio= 2>/dev/null | grep -qE '[0-9]'; then
    echo "RT priority: numeric RTPRIO visible"
  else
    echo "RT priority: none visible"
  fi

  echo
}

PID=""

while [ -z "${PID}" ]; do
  PID="$(find_pid || true)"
  if [ -z "${PID}" ]; then
    sleep "${INTERVAL_SEC}"
  fi
done

echo "Found PID: ${PID}"
echo

while kill -0 "${PID}" 2>/dev/null; do
  print_report "${PID}"
  sleep "${INTERVAL_SEC}"
done

echo "Process ${PID} exited."