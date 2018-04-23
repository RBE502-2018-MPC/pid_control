#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
from rc_node.msg import car_input
import tf
from math import pi, sin, cos, sqrt
from tf.transformations import quaternion_from_euler, euler_from_quaternion

vel_des = 0.75

curr_pose = Pose()

# Gets the desired x,y point at time t
def circle_traj_pt(t):
    circle_dia = 5.0
    offset = (0,0)
    circle_vel = pi*circle_dia/vel_des
    return (circle_dia/2*cos(2*pi/circle_vel*t),
            circle_dia/2*sin(2*pi/circle_vel*t))

def on_new_pose(data):
    global curr_pose
    curr_pose = data.pose

def clamp(val, min, max):
    if val>max:
        return max
    if val<min:
        return min
    return val

def main():
    pub = rospy.Publisher("/car_input", car_input, queue_size=10)
    sub_pose = rospy.Subscriber("/vrpn_client_node/rc_car/pose", PoseStamped, on_new_pose)

    rospy.init_node("pid_control")
    rate = rospy.Rate(100)

    car_msg = car_input()

    despose_broadcaster = tf.TransformBroadcaster()
    err_broadcaster = tf.TransformBroadcaster()

    start_time = rospy.get_rostime()

    while not rospy.is_shutdown():
        time = rospy.get_rostime()

        # Get the desired point for the trajectory
        des_pt = circle_traj_pt((time-start_time).to_sec())

        # Get the current point of the vehicle
        curr_pt = (curr_pose.position.x, curr_pose.position.y)

        # Unit vector in direction of current heading
        curr_euler = euler_from_quaternion((curr_pose.orientation.x,curr_pose.orientation.y,curr_pose.orientation.z,curr_pose.orientation.w))
        curr_hdg = curr_euler[2]

        # Transform the desired point to the current frame
        err_vec = (des_pt[0]-curr_pt[0], des_pt[1]-curr_pt[1])
        err_vec = (err_vec[0]*cos(curr_hdg)+err_vec[1]*sin(curr_hdg),
                  err_vec[0]*-sin(curr_hdg)+err_vec[1]*cos(curr_hdg))

        despose_broadcaster.sendTransform(
            (des_pt[0], des_pt[1], 0),
            quaternion_from_euler(0,0,0),
            time,
            "des_pose",
            "world"
        )

        err_broadcaster.sendTransform(
            #(err[0], err[1], 0),
            (err_vec[0], err_vec[1], 0),
            quaternion_from_euler(0,0,0),
            time,
            "err_pose",
            "rc_car"
        )

        print(err_vec)
        car_msg.steer_angle = clamp(err_vec[1]*200, -60, 60)
        car_msg.power = err_vec[0]*2
        pub.publish(car_msg)

        rate.sleep()

        if((time-start_time).to_sec()>40.0):
            return

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass