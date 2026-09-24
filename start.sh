#!/usr/bin/env bash
# One command to bring up the whole SAR demo:
#   dashboard server -> Gazebo + ArduPilot SITL + RViz + mission bridge
#   -> live feed (person detector) -> dashboard opened in the browser.
#
#   ./start.sh            post-disaster world (default)
#   ./start.sh runway     flat runway world
#
# Then press "Start Search & Rescue" in the dashboard. Ctrl+C here stops
# everything (sim, bridge, detector, server).
set -u

WORLD="${1:-disaster}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URL="http://localhost:5000"

case "$WORLD" in
  disaster|runway) ;;
  *) echo "usage: $0 [disaster|runway]" >&2; exit 1 ;;
esac

if curl -s -o /dev/null "$URL"; then
  echo "Something is already serving $URL (a dashboard already running?). Stop it first." >&2
  exit 1
fi

python3 "$ROOT/command_center/backend/server.py" &
SERVER=$!

cleanup() {
  trap - INT TERM EXIT
  echo
  echo "Stopping simulation, bridge and detector..."
  curl -s -X POST "$URL/api/sim/stop" > /dev/null
  # SIGINT, not SIGTERM: rclpy's handler catches SIGTERM and only stops
  # the server's ROS thread, leaving Flask running.
  kill -INT "$SERVER" 2> /dev/null
  for _ in 1 2 3 4 5; do
    kill -0 "$SERVER" 2> /dev/null || break
    sleep 1
  done
  kill -9 "$SERVER" 2> /dev/null
  wait "$SERVER" 2> /dev/null
  echo "Stopped."
}
trap cleanup INT TERM EXIT

echo "Waiting for the dashboard server..."
until curl -s -o /dev/null "$URL"; do
  if ! kill -0 "$SERVER" 2> /dev/null; then
    echo "Dashboard server exited during startup (see the errors above)." >&2
    exit 1
  fi
  sleep 1
done

echo "Launching the simulation ($WORLD world)..."
curl -s -X POST -H "Content-Type: application/json" -d "{\"world\": \"$WORLD\"}" "$URL/api/sim/launch"
echo
echo "Starting the live feed (person detector)..."
curl -s -X POST "$URL/api/feed/start"
echo

xdg-open "$URL" > /dev/null 2>&1 || true
echo
echo "Dashboard: $URL  (Gazebo and RViz open in their own windows; allow ~30 s)"
echo "Press Start Search & Rescue there. Ctrl+C here stops everything."
wait "$SERVER"
