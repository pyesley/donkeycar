import matplotlib

matplotlib.use("Agg")  # Use an off-screen Agg backend
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import cv2

class PosePlotter:
    """
    This class collects and plots the (x, y) positions of the robot
    over time, as estimated by a Pose class, and returns the plot as an image.
    """

    def __init__(self, width=3, height=3, dpi=100):
        # Lists to store the path
        self.x_data = []
        self.y_data = []

        # Create a matplotlib figure and a canvas using the Agg backend
        self.fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        self.canvas = FigureCanvas(self.fig)

        # If you have headings, you can store them in e.g. self.theta_data = []

    def update(self, pose_x, pose_y, pose_theta):
        """
        Add the latest pose (x, y, theta) to our record of positions.
        """
        self.x_data.append(pose_x)
        self.y_data.append(pose_y)
        # self.theta_data.append(pose_theta)  # if you want heading

    def get_plot_image(self):
        """
        Draw the current trajectory into a matplotlib figure,
        then convert that figure into a NumPy BGR image so it can
        be displayed/overlayed with OpenCV.

        Returns:
            np.ndarray (H x W x 3) in BGR color.
        """
        # Clear the axes before drawing
        self.ax.clear()

        # Plot the trajectory
        self.ax.plot(self.x_data, self.y_data, 'b-', label='Trajectory')
        if len(self.x_data) > 0:
            self.ax.plot(self.x_data[-1], self.y_data[-1], 'ro', label='Current Position')

        self.ax.set_aspect('equal', 'box')
        self.ax.set_xlabel('X position (m)')
        self.ax.set_ylabel('Y position (m)')
        self.ax.set_title('Robot Trajectory')
        self.ax.grid(True)
        self.ax.legend(loc='upper right')

        # Render the figure to the canvas
        self.canvas.draw()

        # Convert to a NumPy RGB array
        buf = np.frombuffer(self.canvas.tostring_rgb(), dtype=np.uint8)
        w, h = self.canvas.get_width_height()
        buf = buf.reshape(h, w, 3)  # h rows, w cols, 3 channels (RGB)

        # Convert RGB->BGR for OpenCV
        plot_bgr = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
        return plot_bgr
