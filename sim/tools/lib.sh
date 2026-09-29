# Shared helpers for the smoke tests. Source it; don't run it.

# One message from a topic. The ros2 CLI has to start and discover the graph
# before it can print anything, which takes several seconds on a small CI
# runner, so retry with a generous timeout rather than mistake a slow start
# for a dead topic. Prints nothing if the topic never delivers.
echo_once() {  # topic [extra ros2 topic echo args...]
  local out=""
  for _ in 1 2 3; do
    out=$(timeout 15 ros2 topic echo --once "$@" 2>/dev/null || true)
    [ -n "$out" ] && break
  done
  printf '%s' "$out"
}
