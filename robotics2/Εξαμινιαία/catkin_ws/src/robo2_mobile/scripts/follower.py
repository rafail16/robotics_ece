#!/usr/bin/env python3

"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        #my Publishers
        self.distancePub = rospy.Publisher("/distance", Float64, queue_size = 100)
        self.linearvelPub = rospy.Publisher("/linx", Float64, queue_size = 100)
        self.angularvelPub = rospy.Publisher("/angz", Float64, queue_size = 100)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu

        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F

        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL

        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR

        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L

        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R

        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)

    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()
        L1 = 0.018
        L2 = 0.05
        L3 = 0.1
        L4 = 0.2
        stop1 = 0  #robot has reached the wall and starts to turn.
        stop2 = 0  #initial positioning of the robot is over. PD control will now take over.
        begin = 0  #indicates PD control has started
        desired_distance = 0.5
        Kp = 6
        Kd = 16
        error = 0
        speedx = 0.3

        while not rospy.is_shutdown():

            sonar_front = self.sonar_F.range
            sonar_right = self.sonar_R.range
            sonar_frontleft = self.sonar_FL.range
            sonar_frontright = self.sonar_FR.range
            if (stop2 == 0):
                if (stop1 == 0):
                    minimum = min(sonar_front + L3, sonar_frontleft - L1*sqrt(2) + L3*sqrt(2), sonar_frontright - L1*sqrt(2) + L3*sqrt(2))
                    if (minimum < desired_distance):   #reach the desired starting position smoothly
                        self.velocity.linear.x = speedx - 0.3/10
                        speedx = speedx - 0.3/10
                        if(speedx < 0.03):
                            stop1 = 1
                    else:
                        self.velocity.linear.x = 0.3 #go forward until you are close to the wall for the first time
                else:
                    if ((sonar_frontright + (L4-L1)*np.sqrt(2)) > (sonar_right + L4) and sonar_front > 1.9):
                        self.velocity.angular.z = 0.0
                        stop2 = 1
                    else:
                        self.velocity.angular.z = -0.6  #turn until being parallel to the wall

            else: #PD control
                begin = 1
                #calculating distances from point of convergence
                final_right = sqrt((sonar_right + L3)**2 + L3**2)
                final_frontright = sonar_frontright + (L3 - L1)*sqrt(2)
                final_frontleft = sonar_frontleft + (L3 - L1)*sqrt(2)
                final_front  = sonar_front + L3
                f = np.arccos(L3/final_right)
                #calculating triangle heights that equal the distance from the wall
                h1 = (final_frontright*final_right*np.sin(0.75*np.pi-f))/sqrt(final_frontright**2 + final_right**2 - 2*final_frontright*final_right*np.cos(0.75*np.pi-f))
                h2 = (final_frontright*final_front*sqrt(2)/2)/sqrt(final_frontright**2 + final_front**2 - 2*final_frontright*final_front*sqrt(2)/2)
                h3 = (final_frontleft*final_front*sqrt(2)/2)/sqrt(final_frontleft**2 + final_front**2 - 2*final_frontleft*final_front*sqrt(2)/2)
                distance = min(h1,h2,h3)
                previous_error = error
                #error for PD control
                error = min(h1, h2 - 0.12, h3 - 0.38) - desired_distance
                self.velocity.linear.x = 0.3
                self.velocity.angular.z = Kp*error + Kd*(error-previous_error)
                angz = self.velocity.angular.z


            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9

            # Publish the new joint's angular positions
            self.velocity_pub.publish(self.velocity)
            linx = self.velocity.linear.x
            #use my publishers
            self.linearvelPub.publish(linx)
            if (begin == 1):
                self.distancePub.publish(distance)
                self.angularvelPub.publish(angz)

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_follower(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
