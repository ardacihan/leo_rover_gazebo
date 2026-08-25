#!/usr/bin/env bash
source "$(dirname "$0")/env.sh"
cd "$HOME/leo_nav2_ws"
exec python3 odom_relay.py
