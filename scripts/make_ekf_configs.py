#!/usr/bin/env python3
"""Generate per-rover EKF configs from the single-robot ekf_leo.yaml template.

`ekf_leo.yaml` is written for leo1 and hardcodes leo1 frames and topics. Two
rovers need one filter each, in their own namespaces, and hand-maintaining two
near-identical 90-line covariance files is how they drift apart. So: substitute
the namespace and write leo1/leo2 variants from the one reviewed template.

Why this matters at all -- the realism harness fed slam_toolbox **raw wheel
odometry**, whose yaw carries a 12% systematic scale error on a skid-steer.
That is harsher than the real rover, which runs exactly this EKF and takes yaw
rate from the gyro instead. In the 2026-08-23 Phase 1 runs it cost leo2's SLAM
its heading entirely (~114 degrees out, map shattered), which took the tag
alignment down with it.
"""

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, 'ekf_leo.yaml')


def main():
    text = open(TEMPLATE).read()
    for ns in ('leo1', 'leo2'):
        out = re.sub(r'\bleo1\b', ns, text)
        # `/**` rather than `ekf_filter_node`: the node is launched into the
        # rover's namespace, so its fully-qualified name is
        # /leo1/ekf_filter_node and a bare `ekf_filter_node:` key silently
        # matches nothing -- the filter would come up with stock defaults, no
        # IMU input, and no indication anything was wrong.
        out = out.replace('ekf_filter_node:', '/**:')
        path = os.path.join(HERE, f'ekf_{ns}.yaml')
        with open(path, 'w') as fh:
            fh.write(out)
        print(f'wrote {path}')
        for key in ('odom_frame', 'base_link_frame', 'odom0:', 'imu0:'):
            for line in out.splitlines():
                if line.strip().startswith(key):
                    print(f'    {line.strip()}')
                    break


if __name__ == '__main__':
    main()
