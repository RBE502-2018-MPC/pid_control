#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion
from rc_node.msg import car_input
import tf
from math import pi, sin, cos, sqrt
from tf.transformations import quaternion_from_euler, euler_from_quaternion

vel_des = 0.75

is_sim = False

curr_header = Header()
curr_pose = Pose()

# Gets the desired x,y point at time t
def circle_traj_pt(t):
    circle_dia = 1.25
    offset = (0.75,0.75)
    circle_vel = pi*circle_dia/vel_des
    return (circle_dia/2*cos(2*pi/circle_vel*t) + offset[0],
            circle_dia/2*sin(2*pi/circle_vel*t) + offset[1])

def on_new_pose(data):
    global curr_pose
    global curr_header
    if not is_sim:
        # Need to rotate the heading because the Vicon sets an arbitrary one
        ori = data.pose.orientation
        eulers = euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
        quat = quaternion_from_euler(eulers[0], eulers[1], eulers[2]-107*pi/180)
        data.pose.orientation = Quaternion(*quat)
    curr_header = data.header
    curr_pose = data.pose
    print curr_pose

def clamp(val, min, max):
    if val>max:
        return max
    if val<min:
        return min
    return val

def main():
    pub = rospy.Publisher("/car_input", car_input, queue_size=10)
    sub_pose = rospy.Subscriber("/vrpn_client_node/rc_car/pose", PoseStamped, on_new_pose)
    pubfix = rospy.Publisher("/pose_fixed", PoseStamped, queue_size=10)

    rospy.init_node("pid_control")
    rate = rospy.Rate(100)

    car_msg = car_input()
    pose_msg = PoseStamped()

    despose_broadcaster = tf.TransformBroadcaster()
    err_broadcaster = tf.TransformBroadcaster()
    fix_broadcaster = tf.TransformBroadcaster()

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
            (err_vec[0], err_vec[1], 0),
            quaternion_from_euler(0,0,0),
            time,
            "err_pose",
            "rc_car_fixed"
        )

        #print(err_vec)
        car_msg.steer_angle = clamp(err_vec[1]*75, -40, 40)
        car_msg.power = (err_vec[0] - 0.1)*0.7
        print(car_msg)
        pub.publish(car_msg)

        if not is_sim:
            # Republish the fixed heading for better updates
            pose_msg.header = curr_header
            pose_msg.pose = curr_pose
            pubfix.publish(pose_msg)
            fix_broadcaster.sendTransform(
                (curr_pose.position.x, curr_pose.position.y, curr_pose.position.z),
                (curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z, curr_pose.orientation.w),
                time,
                "rc_car_fixed",
                "world"
            )

        '''if ((time - start_time).to_sec()>10.0):
            car_msg.steer_angle=0
            car_msg.power=0
            for i in range(10):
              pub.publish(car_msg)
              rate.sleep()
            return'''
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass