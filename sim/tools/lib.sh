# Shared helpers for the smoke tests. Source it; don't run it.

# One message from a topic. The ros2 CLI has to start and discover the graph
# before it can print anything, which takes several seconds on a small CI
# runner, so retry with a generous timeout rather than mistake a slow start
# for a dead topic. Pass the message type: without it the CLI has to look the
# type up through the ros2 daemon, and a stale daemon makes echo exit at once
# with nothing. A whole message ends with a `---` line; output without one is
# a print the timeout cut off halfway, so it doesn't count. Prints nothing if
# the topic never delivers.
echo_once() {  # topic [type] [extra ros2 topic echo args...]
  local out=""
  for _ in 1 2 3 4; do
    out=$(timeout 20 ros2 topic echo --once "$@" 2>/dev/null || true)
    printf '%s\n' "$out" | grep -qx -- '---' && break
    out=""
    sleep 2
  done
  printf '%s' "$out"
}
