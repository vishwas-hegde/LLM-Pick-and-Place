#!/usr/bin/env python3

import argparse
import os
import random
import time
import xacro
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy
import math

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (in radians) to a quaternion."""
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return qx, qy, qz, qw

def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Continuously spawn objects into our Gazebo world at a fixed location.')
    parser.add_argument('--package', type=str, default='ros2_grasping', help='Package where URDF/XACRO files are located.')
    parser.add_argument('--name', type=str, default='box', help='Base name of the objects to spawn.')
    parser.add_argument('--x', type=float, default=0.5, help='the x component of the initial position [meters].')
    parser.add_argument('--y', type=float, default=-0.5, help='the y component of the initial position [meters].')
    parser.add_argument('--z', type=float, default=1.5, help='the z component of the initial position [meters].')
    parser.add_argument('--yaw_min', type=float, default=0.0, help='Minimum yaw angle (in degrees) for randomization.')
    parser.add_argument('--yaw_max', type=float, default=360.0, help='Maximum yaw angle (in degrees) for randomization.')

    args, unknown = parser.parse_known_args()

    try:
        package_share_directory = get_package_share_directory(args.package)
    except Exception as e:
        print(f"Error: Could not find package `{args.package}`. Ensure the package is built and sourced.")
        return

    urdf_paths = {
        # 'orange_box': os.path.join(package_share_directory, 'urdf', 'orange_box.urdf'),
        # 'green_box': os.path.join(package_share_directory, 'urdf', 'green_box.urdf'),
        # 'cuboid': os.path.join(package_share_directory, 'urdf', 'cuboid.urdf'),
        # 'cylinder': os.path.join(package_share_directory, 'urdf', 'cylinder.urdf'),
        # 'prism': os.path.join(package_share_directory, 'urdf', 'prism.urdf'),
        'tomato_soup_can': os.path.join(package_share_directory,'urdf', 'ycb_assets', '005_tomato_soup_can.urdf')
    }

    # Start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    # Create client to communicate with `/spawn_entity` service
    node.get_logger().info('Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')
    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available! Exiting...')
        node.destroy_node()
        rclpy.shutdown()
        return

    spawn_count = 0
    try:
        while rclpy.ok():
            # Randomly choose which box to spawn
            # color_choice = random.choice(['orange', 'green'])
            # For 80% orange, 20% green:
            # color_choice = random.choices(['orange_box', 'green_box'], weights=[0.8, 0.2])[0]
            # Randomly choose an object type to spawn
            object_choice = random.choice(list(urdf_paths.keys()))
            # urdf_file_path = urdf_paths[object_choice]
            # urdf_file_path = "/home/sarthak_m/VBRM/Projects/ConveyorGrasp/src/ros2_grasping/urdf/ycb_assets/005_tomato_soup_can.urdf"
            # Use package-relative path
            # urdf_file_path = os.path.join(package_share_directory, 'urdf', 'ycb_assets', '005_tomato_soup_can.urdf')
            urdf_file_path = os.path.join(get_package_share_directory('ros2_grasping'), 'urdf/ycb_assets/005_tomato_soup_can.urdf')

            print(urdf_file_path)
            # Check if the URDF file exists
            if not os.path.exists(urdf_file_path):
                node.get_logger().error(f"URDF file `{urdf_file_path}` not found! Skipping spawn.")
                continue

            # Set up spawn request
            request = SpawnEntity.Request()
            request.name = f"{args.name}_{object_choice}_{spawn_count}"  # Unique name for each spawn

            # Read and process the URDF file
            try:
                with open(urdf_file_path, 'r') as file:
                    request.xml = file.read()
            except Exception as e:
                node.get_logger().error(f"Failed to read URDF file `{urdf_file_path}`: {e}")
                continue

            # Set fixed position
            request.initial_pose.position.x = args.x
            request.initial_pose.position.y = args.y
            request.initial_pose.position.z = args.z

            # Generate a random yaw angle within the specified range
            random_yaw = random.uniform(math.radians(args.yaw_min), math.radians(args.yaw_max))

            # Convert yaw angle to quaternion and set orientation
            qx, qy, qz, qw = euler_to_quaternion(0, 0, random_yaw)
            request.initial_pose.orientation.x = qx
            request.initial_pose.orientation.y = qy
            request.initial_pose.orientation.z = qz
            request.initial_pose.orientation.w = qw

            # Spawn the object
            node.get_logger().info(f"Spawning `{request.name}` ({object_choice}) at ({args.x}, {args.y}, {args.z}) with random yaw={math.degrees(random_yaw):.2f} degrees")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)

            if future.result() is not None:
                node.get_logger().info(f"Successfully spawned `{request.name}`")
            else:
                node.get_logger().error(f"Failed to spawn `{request.name}`")

            # Increment counter and pause before next spawn
            spawn_count += 1
            time.sleep(5)  # Delay of 20 secs before spawning the next object

    except KeyboardInterrupt:
        node.get_logger().info('Spawning interrupted by user.')
    finally:
        node.get_logger().info('Shutting down node.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()