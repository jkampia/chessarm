import math as m
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

from kinematic_helper import ARM_5DOF

class InteractiveDisplay:


    def __init__(self, link_lengths=None, num_joints=5):


        self.num_joints = num_joints
        self.link_lengths = link_lengths if link_lengths else [4, 4, 3, 2, 2]
        self.joint_angles = [0] * self.num_joints

        # Setup plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(bottom=0.35)

        # Declare arm with initial angles
        joint_params = [10, 10, 10, 10, 10]
        init_angles = [0.0, m.pi/2, 0.0, 0.0, 0.0]
        self.arm = ARM_5DOF(joint_params, init_angles)

        # Draw initial robot arm
        self.positions = self.compute_positions()
        self.arm_line, = self.ax.plot(self.positions[:, 0], self.positions[:, 1], self.positions[:, 2],
                                      color='black', linewidth=2)
        self.joint_scatters = [self.ax.scatter(*pos, s=100) for pos in self.positions]

        self.setup_plot_limits()
        self.create_sliders()
        self.connect_sliders()

        plt.show()
    

    def setup_plot_limits(self):
        """Set consistent limits and labels for 3D plot."""
        total_length = sum(self.link_lengths)
        self.ax.set_xlim(-total_length, total_length)
        self.ax.set_ylim(-total_length, total_length)
        self.ax.set_zlim(-5, 5)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("Interactive 3D Robot Arm")


    def create_sliders(self):
        """Create a list of sliders for each joint angle."""
        self.sliders = []
        for i in range(self.num_joints):
            ax_slider = plt.axes([0.15, 0.25 - i * 0.05, 0.7, 0.03])
            slider = Slider(ax_slider, f'Joint {i+1}', -180, 180, valinit=0)
            self.sliders.append(slider)


    def connect_sliders(self):
        """Connect slider updates to robot arm redraw."""
        for slider in self.sliders:
            slider.on_changed(self.update_plot)


    def update_plot(self, val):
        """Redraw robot arm based on updated slider values."""
        self.joint_angles = [slider.val for slider in self.sliders]
        self.positions = self.compute_positions()

        self.arm_line.set_data(self.positions[:, 0], self.positions[:, 1])
        self.arm_line.set_3d_properties(self.positions[:, 2])
        for i, pos in enumerate(self.positions):
            self.joint_scatters[i]._offsets3d = ([pos[0]], [pos[1]], [pos[2]])

        self.fig.canvas.draw_idle()
