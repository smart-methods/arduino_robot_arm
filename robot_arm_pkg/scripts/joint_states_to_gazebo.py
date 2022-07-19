#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
    pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=10)

    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names = ['base_joint', 'shoulder', 'elbow', 'wrist']
    point=JointTrajectoryPoint()
    point.positions = [data.position[0], data.position[1], data.position[2], data.position[3]]
    point.time_from_start = rospy.Duration(2)
    joints_str.points.append(point)

    pub.publish(joints_str)
    rospy.loginfo("position updated")



def listener():
    rospy.init_node('states', anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)

    rospy.spin()



if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
