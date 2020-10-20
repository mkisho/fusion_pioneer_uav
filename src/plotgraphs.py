#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from matplotlib.animation import FuncAnimation
import tf
from tf.transformations import quaternion_matrix


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 10000)
        self.ax.set_ylim(-7, 7)
        return self.ln

    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] 
        return yaw   

    def robot_odom(self, msg):
        yaw_angle = self.getYaw(msg.pose.pose)
        self.y_data.append(yaw_angle)
        x_index = len(self.x_data)
        self.x_data.append(x_index+1)

    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln



def poseStamped(msg):
    global counter


def bebop_odom(msg):
     counter
def destiny(msg):
     counter


if __name__ == '__main__':
    counter = 0

    vis = Visualiser()
    rospy.init_node("plotter")
    rospy.Subscriber("/bebop/poseStamped", PoseStamped, poseStamped)
    rospy.Subscriber("/bebop/odometry/filtered", Odometry, vis.robot_odom)
    rospy.Subscriber("/bebop/odom", Odometry, bebop_odom)
    rospy.Subscriber("/goal", Point, destiny)
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True)
    rospy.spin()