#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import the node classes from your existing scripts
# Make sure these files are in the same directory or accessible in your Python path
from arduino_ros_bridge import ArduinoBridgeNode #
from camerastream import GstCameraPublisher     #

def main(args=None):
    rclpy.init(args=args)

    arduino_node = None
    camera_node = None
    executor = None

    try:
        # Instantiate both nodes
        arduino_node = ArduinoBridgeNode() #
        camera_node = GstCameraPublisher() #

        # Create a MultiThreadedExecutor
        # You can adjust num_threads, default uses thread pool size based on CPU cores
        executor = MultiThreadedExecutor()

        # Add both nodes to the executor
        executor.add_node(arduino_node)
        executor.add_node(camera_node)

        print("Nodes added to executor. Spinning...")
        # Spin the executor, processing callbacks for both nodes
        executor.spin()

    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Shutting down...")
    except Exception as e:
        print(f"An error occurred: {e}")
        # Optional: Log the full traceback
        # import traceback
        # traceback.print_exc()
    finally:
        # Cleanup
        if executor is not None:
            # This should implicitly stop spinning if not already stopped
            executor.shutdown()

        if camera_node is not None and rclpy.ok(): # Check if rclpy is still valid
             # Ensure camera resources are released if possible
             # The camera node's destroy_node method handles cap.release()
            camera_node.destroy_node()

        if arduino_node is not None and rclpy.ok():
            # The arduino node's shutdown method handles serial closing and thread joining
            arduino_node.shutdown()
            arduino_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
        print("Cleanup complete.")


if __name__ == '__main__':
    main()