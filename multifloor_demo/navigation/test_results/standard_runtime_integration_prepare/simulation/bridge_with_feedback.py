#!/usr/bin/env python3
"""Test-only aggregate health for required body feedback; production untouched."""
import json
import os
import pathlib
import sys
import time

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros
from std_msgs.msg import String

SIM = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(SIM))
from control_bridge import Bridge


class FeedbackHealth:
    def __init__(self):
        self.data=None
        self.wall=None
        self.failure=None
        self.established=False

    def update(self, data, wall):
        # Check elapsed silence before replacing its timestamp with late data.
        self.state(wall)
        if (not isinstance(data,dict) or data.get('state') not in
                ('boot','waiting','disabled','active','neutral_for_stop','failed')
                or type(data.get('enabled')) is not bool
                or type(data.get('failed')) is not bool
                or (data.get('state')=='disabled' and data.get('enabled'))):
            self.failure=self.failure or 'invalid_body_feedback_status'
            return
        self.data=dict(data)
        self.wall=wall
        self.established |= data['state'] in ('disabled','active','neutral_for_stop')
        if data['failed'] or data['state']=='failed':
            self.failure=self.failure or str(data.get('reason') or 'body_feedback_failed')

    def state(self, now):
        if self.established and self.wall is not None and now-self.wall>.30:
            self.failure=self.failure or 'body_feedback_status_timeout'
        if self.failure:return 'failed', self.failure
        if self.wall is None or now-self.wall>.30:return 'hold','body_feedback_status_stale'
        if self.data['state'] not in ('disabled','active','neutral_for_stop'):
            return 'hold','body_feedback_not_ready'
        return 'ready','body_feedback_ready'


class StatusPublisher:
    """Preserve every production field and append this required module's health."""
    def __init__(self, publisher, health):self.publisher=publisher;self.health=health
    def publish(self, msg):
        data=json.loads(msg.data)
        data.update(body_feedback_required=True,body_feedback=self.health.data,
                    body_feedback_wall_age=None if self.health.wall is None else time.monotonic()-self.health.wall)
        self.publisher.publish(String(data=json.dumps(data)))


class FeedbackBridge(Bridge):
    def __init__(self):
        self.feedback_health=FeedbackHealth()
        super().__init__()
        self.status=StatusPublisher(self.status,self.feedback_health)
        self.create_subscription(String,'/demo/test/body_stabilizer/status',self.on_feedback,10)

    def state(self, now):
        state,reason=super().state(now)
        other,detail=self.feedback_health.state(now)
        if state=='failed':return state,reason
        if other=='failed':return other,detail
        if state!='ready':return state,reason
        return other,detail if other!='ready' else reason

    def on_feedback(self, msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):data=None
        now=time.monotonic();self.feedback_health.update(data,now)
        if self.state(now)[0]!='ready':self.stop()
        self.report()


def main():
    rclpy.init();node=FeedbackBridge()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    except RuntimeError:
        # A SIGINT may invalidate the context while rclpy takes a queued DDS
        # sample. Never suppress a runtime failure of a still-live context.
        if rclpy.ok():raise
    finally:
        try:
            if rclpy.ok():node.stop()
        except (_ros.RCLError,rclpy.executors.ExternalShutdownException):
            if rclpy.ok():raise
        finally:
            node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
