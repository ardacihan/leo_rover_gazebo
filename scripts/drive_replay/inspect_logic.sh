#!/bin/bash
DIR=${1:-/tmp/logic_test}
ls -la "$DIR"
python3 - "$DIR" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1] + '/logic.json'))
print('goals:', d['goals'])
print('shadow msgs', len(d['shadow']), 'cmd msgs', len(d['cmd']))
for e in d['events'][:30]:
    print(e)
EOF
cp "$DIR/map_final.png" /mnt/c/Users/smirn/Desktop/leo_rover_gazebo/reports/drive_2026-08-20/_maptest.png
