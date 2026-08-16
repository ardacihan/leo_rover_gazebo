#!/usr/bin/env python3
"""Record the shared item registry + explorer status streams to JSONL.

Every /item_claims and /leo{i}/frontier_explorer/status message is appended
as one JSON line stamped with sim time. The item-search benchmark metrics
(time-to-k-items, time-to-all-items) are derived offline from this log.

Usage: python3 item_recorder.py <out.jsonl> <robot1,robot2,...>
"""

import json
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String as StringMsg


class ItemRecorder(Node):
    def __init__(self, path, robots):
        super().__init__('item_recorder')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.f = open(path, 'a', buffering=1)
        self.create_subscription(
            StringMsg, '/item_claims',
            lambda m: self._log('item_claims', m), 50)
        for r in robots:
            self.create_subscription(
                StringMsg, f'/{r}/frontier_explorer/status',
                lambda m, r=r: self._log(f'status/{r}', m), 10)

    def _log(self, topic, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            payload = {'raw': msg.data}
        rec = {'t': round(self.get_clock().now().nanoseconds / 1e9, 2),
               'topic': topic, **payload}
        self.f.write(json.dumps(rec) + '\n')


def main():
    path = sys.argv[1]
    robots = sys.argv[2].split(',') if len(sys.argv) > 2 else ['leo1', 'leo2']
    rclpy.init()
    node = ItemRecorder(path, robots)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
