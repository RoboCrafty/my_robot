#!/bin/bash
# Runs the simulated controller and the web server; exits if either dies so the host restarts the container.
cd /app/rpi-controller
./build/parolController --sim < /dev/null &
python3 web/server.py &
wait -n
exit 1
