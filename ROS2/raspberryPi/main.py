#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import the node classes from your existing scripts
from arduino_ros_bridge import ArduinoBridgeNode #
from camerastream import GstCameraPublisher     #
from IMU_6050 import MPU6050PublisherNode  # New IMU Node

def main(args=None): #
    rclpy.init(args=args)

    arduino_node = None
    camera_node = None
    imu_node = None # Added IMU node
    executor = None

    try:
        # Instantiate all nodes
        arduino_node = ArduinoBridgeNode() #
        camera_node = GstCameraPublisher() #
        imu_node = MPU6050PublisherNode()  # Instantiate the new IMU node

        executor = MultiThreadedExecutor() #

        # Add all nodes to the executor
        executor.add_node(arduino_node) #
        executor.add_node(camera_node) #
        executor.add_node(imu_node)    # Add the new IMU node

        print("Nodes added to executor. Spinning...")
        executor.spin() #

    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Shutting down...") #
    except Exception as e:
        print(f"An error occurred: {e}") #
    finally:
        if executor is not None:
            executor.shutdown() #

        # It's good practice to destroy nodes in reverse order of creation or based on dependencies
        # For now, order might not strictly matter as much, but consider if inter-node dependencies exist.

        if imu_node is not None and rclpy.ok():
            imu_node.shutdown() # Custom shutdown for MPU6050 node
            imu_node.destroy_node()
            print("MPU6050 node destroyed.")

        if camera_node is not None and rclpy.ok(): #
            # camerastream.py needs a destroy_node method that calls cap.release()
            # Assuming GstCameraPublisher has a proper destroy_node or shutdown method
            if hasattr(camera_node, 'shutdown'): # Check if custom shutdown exists
                camera_node.shutdown()
            camera_node.destroy_node() #
            print("Camera node destroyed.")

        if arduino_node is not None and rclpy.ok(): #
            arduino_node.shutdown() #
            arduino_node.destroy_node() #
            print("Arduino node destroyed.")

        if rclpy.ok():
            rclpy.shutdown() #
        print("Cleanup complete.")


if __name__ == '__main__':
    main()