#!/usr/bin/env bash
# Self-safe full stack restart for jetson-04 (~/leo_nav2_ws). Run detached:
#   setsid nohup bash full_restart.sh < /dev/null > /dev/null 2>&1 &
# pkill patterns are bracketed/path-anchored: plain 'nav2' matches the
# invoking shell's own cmdline and every leo_nav2_ws path (self-kill).
cd $HOME/leo_nav2_ws
exec > logs/full_restart.log 2>&1
pkill -f 'explore_lit[e]'
pkill -f 'aruco_detecto[r]'
pkill -f 'run_recorder.p[y]'
pkill -f 'cam_sampler.p[y]'
pkill -f 'real_navigation.launch.p[y]'
sleep 4
pkill -9 -f '/opt/ros/humble/lib/nav[2]'
pkill -9 -f 'slam_toolbo[x]'
pkill -f 'tf_freshener.p[y]'
pkill -f 'scan_to_scan_filter_chai[n]'
pkill -f 'run_scanfilter.s[h]'
sleep 3
rm -f logs/stack.pid
bash start_stack.sh
setsid nohup bash run_freshener.sh > logs/freshener.log 2>&1 < /dev/null &
setsid nohup bash run_scanfilter.sh > logs/scanfilter.log 2>&1 < /dev/null &
echo RESTART_SCRIPT_DONE
