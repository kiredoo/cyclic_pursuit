"""Real ROS 2 topic integration test with simulated inputs, without Crazyswarm2.

This test instantiates only ROS 2 control nodes. No Crazyflie drivers, radio,
flight server, or hardware are started.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from cyclic_pursuit.controller import Crazyflie


AGENTS = ('cf231', 'cf123', 'cf232')


def drive(executor, seconds, tick=None):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if tick is not None:
            tick()
        executor.spin_once(timeout_sec=0.015)


def pose(name, x, y, yaw=0.0, valid=True):
    msg = PoseStamped()
    msg.header.frame_id = 'world'
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(yaw / 2) if valid else 0.0
    msg.pose.orientation.w = math.cos(yaw / 2) if valid else 0.0
    return msg


def test_three_agent_topics_and_missing_pose_safety():
    rclpy.init()
    executor = SingleThreadedExecutor()
    controllers = []
    observer = None
    try:
        for name in AGENTS:
            controllers.append(Crazyflie(
                name, name.removeprefix('cf'), AGENTS,
                0.3, 0.7853981633974483, 1.0, 4.0,
                5.497787143782138, 2.83, 2.0, 2.0,
                0.17, 0.31, 0.8, len(AGENTS), 0.5,
            ))

        observer = Node('integration_test_observer')
        publishers = {
            name: observer.create_publisher(PoseStamped, f'/{name}/pose', 10)
            for name in AGENTS
        }
        received = {name: [] for name in AGENTS}
        subscriptions = []
        for name in AGENTS:
            subscriptions.append(observer.create_subscription(
                Twist,
                f'/{name}/cmd_vel_hover',
                lambda msg, target=name: received[target].append(msg),
                10,
            ))
        for node in controllers:
            executor.add_node(node)
        executor.add_node(observer)

        # Wait for discovery so that publication occurs after matching.
        drive(executor, 2.0)
        assert all(pub.get_subscription_count() > 0 for pub in publishers.values())
        assert all(not received[name] for name in AGENTS), (
            'Controller must not emit commands before any measured poses'
        )

        first_two = AGENTS[:2]
        def publish_two():
            publishers[first_two[0]].publish(pose(first_two[0], 1.5, 0.0))
            publishers[first_two[1]].publish(pose(first_two[1], -0.75, 1.2))

        drive(executor, 0.8, publish_two)
        assert all(not received[name] for name in AGENTS), (
            'Controller must wait until every active agent has reported a pose'
        )

        positions = ((1.5, 0.0), (-0.75, 1.2), (-0.75, -1.2))
        def publish_all():
            for name, (x, y) in zip(AGENTS, positions):
                publishers[name].publish(pose(name, x, y))

        drive(executor, 1.5, publish_all)
        assert all(received[name] for name in AGENTS), (
            f'Missing control command from: {[n for n in AGENTS if not received[n]]}'
        )
        for name in AGENTS:
            cmd = received[name][-1]
            assert math.isfinite(cmd.angular.z)
            assert cmd.linear.x == 0.3
            assert cmd.linear.z == 0.5

        # A malformed zero quaternion must disable the affected controller.
        publishers[AGENTS[0]].publish(pose(AGENTS[0], 1.5, 0.0, valid=False))
        drive(executor, 0.15)
        count_after_invalid = len(received[AGENTS[0]])
        drive(executor, 0.25)
        assert len(received[AGENTS[0]]) == count_after_invalid

        # With no new poses, all controllers must stop sending stale commands.
        drive(executor, 1.3)
        last_counts = {name: len(received[name]) for name in AGENTS}
        drive(executor, 0.2)
        assert all(len(received[name]) == last_counts[name] for name in AGENTS)
    finally:
        if observer is not None:
            executor.remove_node(observer)
            observer.destroy_node()
        for node in controllers:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
