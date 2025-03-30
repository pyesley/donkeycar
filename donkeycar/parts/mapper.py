'''
    This code will use the following sensor input
    encoder
    IMU -  YAW
    camera images
    direction of car

    from that information we will produce a real world location of the car
    hence an X,Y position.

    the class will also have a method to produce an image which is a the plot
    of the x,y position
'''

import cv2
import matplotlib.pyplot as plt
from aruduinoSerial import ArduinoSerial
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import io
from PIL import Imageimport
import numpy as np
import logging
import math
from PIL import Image

class Mapper:

        def __init__(self):
            self.xCurrent = 0
            self.yCurrent = 0
            self.yaw = 0
            self.velocity = 0
            self.ticks = 0
            self.last_ticks = 0  # Store previous ticks for distance calculation
            self.encoder = 0
            self.imgLast = None
            self.direction = 1  # Default: forward
            self.arduino = ArduinoSerial()
            self.xHistory = [0]  # Start with initial position
            self.yHistory = [0]


        def updateCameraPosition(img):
            '''
                This method will will take the new image, and compore to the last image, self.imgLast
                use optical odometry to determine the distance the car has traveled
            '''
            # ---------------------
            # Camera Intrinsic Parameters
            # Replace these with your camera's calibrated parameters.
            # fx, fy: focal lengths; cx, cy: principal point.
            fx = 700.32833455
            fy = 703.47654591
            cx = 633.7861054
            cy = 342.84793261

            K = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float64)

            # Distortion coefficients (assume zero or insert your actual distortion from calibration)
            dist_coeffs = [-0.30944433, 0.1258339, -0.00045581, -0.00057164, -0.01383379]

            # ---------------------
            # Function that gets da features of a image and put it on image






        def updateEncoderPosition(encoder):
            '''
                This method will take the encoder value and determine the distance the car has traveled
            '''
            ticks, velocity = ArduinoSerial.request_ENC()
            self.velocity = velocity
            self.ticks = ticks

        def updateIMU(imu):
            '''
                This method will take the imu value and determine the change in yaw
            '''
            roll,pitch,yaw = ArduinoSerial.request_im2()
            if yaw != -999 :
                self.yaw = yaw

        def sensorFusion(self):
            '''
            Combine data from camera, encoder, and IMU to estimate position
            '''
            if hasattr(self, 'last_ticks'):
                # Calculate distance traveled using encoder
                delta_ticks = self.ticks - self.last_ticks
                distance = delta_ticks * 0.01  # Conversion factor from ticks to distance (adjust as needed)

                # Use yaw from IMU for heading in radians
                heading_rad = math.radians(self.yaw)

                # Apply direction (-1 for reverse, 1 for forward)
                effective_distance = distance * self.direction

                # Update position based on heading and distance
                self.xCurrent += effective_distance * math.cos(heading_rad)
                self.yCurrent += effective_distance * math.sin(heading_rad)

                # Store position in history
                self.xHistory.append(self.xCurrent)
                self.yHistory.append(self.yCurrent)

                logging.info(f"Position updated: x={self.xCurrent:.2f}, y={self.yCurrent:.2f}, heading={self.yaw:.2f}°")

            # Update last_ticks for next calculation
            self.last_ticks = self.ticks

        def getPlotImage(self):
            '''
            Generate a visualization of the robot's path
            '''
            # Create figure and axis
            fig = Figure(figsize=(8, 6), dpi=100)
            canvas = FigureCanvasAgg(fig)
            ax = fig.add_subplot(111)

            # Plot trajectory
            ax.plot(self.xHistory, self.yHistory, 'b-', linewidth=2)

            # Mark the current position
            ax.plot(self.xCurrent, self.yCurrent, 'ro', markersize=10)

            # Add start point
            if len(self.xHistory) > 0:
                ax.plot(self.xHistory[0], self.yHistory[0], 'go', markersize=8)

            # Set labels and title
            ax.set_xlabel('X position (m)')
            ax.set_ylabel('Y position (m)')
            ax.set_title('Robot Trajectory')
            ax.grid(True)

            # Equal aspect ratio
            ax.axis('equal')

            # Add margins around the path
            if len(self.xHistory) > 1:
                xmin, xmax = min(self.xHistory), max(self.xHistory)
                ymin, ymax = min(self.yHistory), max(self.yHistory)
                margin = max(0.5, (xmax - xmin) * 0.1, (ymax - ymin) * 0.1)
                ax.set_xlim(xmin - margin, xmax + margin)
                ax.set_ylim(ymin - margin, ymax + margin)
            else:
                ax.set_xlim(-1, 1)
                ax.set_ylim(-1, 1)

            # Draw the plot to the canvas
            canvas.draw()

            # Convert to numpy array
            s, (width, height) = canvas.print_to_buffer()
            plot_image = np.frombuffer(s, np.uint8).reshape((height, width, 4))

            # Convert RGBA to RGB
            plot_image = cv2.cvtColor(plot_image, cv2.COLOR_RGBA2RGB)

            return plot_image

        def run(self, imageNew, direction=1):
            '''
            Main run loop - process new data and update map
            '''
            self.direction = direction  # Update direction (1=forward, -1=reverse)
            self.imgLast = imageNew.copy() if self.imgLast is None else self.imgLast

            self.updateCameraPosition(imageNew)
            self.updateEncoderPosition()  # No need to pass encoder as it's handled inside
            self.updateIMU()
            self.sensorFusion()

            plot_image = self.getPlotImage()

            # Update imgLast for next time
            self.imgLast = imageNew.copy()

            return plot_image

