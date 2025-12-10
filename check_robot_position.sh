#!/bin/bash
# Quick command to check robot's current position
# Usage: ./check_robot_position.sh

    echo "Robot Current Position (from /g1/mpc_observation):"
    echo "=================================================="
    ros2 topic echo /g1/mpc_observation --once 2>/dev/null | \
    grep -A 100 "state:" | \
    head -10 | \
    awk '/value:/ {if (NR==1) print "  X:", $2; if (NR==2) print "  Y:", $2; if (NR==3) print "  Z:", $2; if (NR==6) print "  Yaw:", $2}'

    echo ""
    echo "For continuous monitoring, use:"
    echo "  ros2 topic echo /g1/mpc_observation | grep -A 6 'state:'"



