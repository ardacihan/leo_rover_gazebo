#!/usr/bin/env python3
"""Side-by-side of pass-1 (explorer stopped early) and pass-2 map/global."""
import cv2
import numpy as np

OUT = '/mnt/c/Users/smirn/Desktop/leo_rover_gazebo/reports/drive_2026-08-20/frames/pass1_vs_pass2.png'
R2 = '/mnt/c/Users/smirn/Desktop/leo_rover_gazebo/reports/drive_2026-08-20/drive_2026-08-20/logic'

tiles = []
for path, name in [('/tmp/logic_test/map.mp4', 'pass1 map'),
                   (f'{R2}/map.mp4', 'pass2 map'),
                   ('/tmp/logic_test/global_costmap.mp4', 'pass1 gc'),
                   (f'{R2}/global_costmap.mp4', 'pass2 gc')]:
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 8.0
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(502 * fps))
    ok, f = cap.read()
    cap.release()
    if not ok:
        continue
    s = 430 / max(f.shape[:2])
    f = cv2.resize(f, (int(f.shape[1] * s), int(f.shape[0] * s)))
    cv2.rectangle(f, (0, 0), (f.shape[1], 18), (0, 0, 0), -1)
    cv2.putText(f, f'{name} t=502', (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (255, 255, 255), 1)
    tiles.append(f)
H = max(t.shape[0] for t in tiles)
W = max(t.shape[1] for t in tiles)
sheet = np.zeros((2 * H, 2 * W, 3), np.uint8)
for i, f in enumerate(tiles):
    r, c = divmod(i, 2)
    sheet[r * H:r * H + f.shape[0], c * W:c * W + f.shape[1]] = f
cv2.imwrite(OUT, sheet)
print('wrote', OUT)
