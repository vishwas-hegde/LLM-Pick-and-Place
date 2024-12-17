#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
from gazebo_msgs.srv import SpawnEntity
import rclpy
import numpy as np


def find_model_path(model_name):
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    print(gazebo_model_path)
    for path in gazebo_model_path.split(':'):
        model_dir = os.path.join(path, model_name)
        model_sdf = os.path.join(model_dir, 'model.sdf')
        if os.path.exists(model_sdf):
            return model_sdf
    raise FileNotFoundError(f"Model `{model_name}` not found in GAZEBO_MODEL_PATH.")


def main():
    parser = argparse.ArgumentParser(description='Spawn object into our Gazebo world.')
    parser.add_argument('--name', default='can', type=str, required=True, help='Name of the model to spawn.')
    parser.add_argument('--namespace', type=str, default='ros2Grasp', help='ROS namespace for tf and plugins.')
    parser.add_argument('--x', type=float, default=0.5, help='The x component of the initial position [meters].')
    parser.add_argument('--y', type=float, default=-0.5, help='The y component of the initial position [meters].')
    parser.add_argument('--z', type=float, default=0.5, help='The z component of the initial position [meters].')
    parser.add_argument('--num', type=int, default=1, help='Number of objects to spawn.')
    parser.add_argument('--ox', type=float, default=0.0, help='The x component of the initial orientation [quaternion].')
    parser.add_argument('--oy', type=float, default=0.0, help='The y component of the initial orientation [quaternion].')
    parser.add_argument('--oz', type=float, default=0.0, help='The z component of the initial orientation [quaternion].')

    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('entity_spawner')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Waiting for `/spawn_entity` service...')
    client.wait_for_service()
    node.get_logger().info('...connected!')

    # Locate and read the model SDF file
    try:
        model_sdf_path = find_model_path(args.name)
        with open(model_sdf_path, 'r') as sdf_file:
            sdf_xml = sdf_file.read()
    except FileNotFoundError as e:
        node.get_logger().error(str(e))
        return
    
    name = args.name # + str(np.random.randint(0, 1000))
    name = name + '_' + str(args.num)
    request = SpawnEntity.Request()
    request.name = name
    # request.namespace = args.namespace
    request.xml = sdf_xml
    request.initial_pose.position.x = args.x
    request.initial_pose.position.y = args.y
    request.initial_pose.position.z = args.z
    request.initial_pose.orientation.x = args.ox
    request.initial_pose.orientation.y = args.oy
    request.initial_pose.orientation.z = args.oz

    node.get_logger().info(f"Spawning `{args.name}` at ({args.x}, {args.y}, {args.z})...")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Successfully spawned `{args.name}`.")
    else:
        node.get_logger().error(f"Failed to spawn `{args.name}`: {future.exception()}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
