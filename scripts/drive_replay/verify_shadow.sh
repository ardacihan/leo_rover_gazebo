#!/bin/bash
DIR=${1:?replay out dir}
grep -c 'Begin navigating' "$DIR/stack.log" | sed 's/^/goals: /'
grep -m2 'Guarding' "$DIR/stack.log"
grep -c 'Exploration stopped' "$DIR/stack.log" | sed 's/^/explore stopped: /'
python3 - "$DIR" <<'EOF'
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
with AnyReader([Path(sys.argv[1]) / 'shadow_bag'],
               default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as r:
    for c in sorted(r.connections, key=lambda c: c.topic):
        print(f'{c.topic:45s} {c.msgcount}')
EOF
