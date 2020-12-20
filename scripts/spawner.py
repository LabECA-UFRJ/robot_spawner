#!/usr/bin/env python
import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from simulation_msgs.msg import Robot

robot = None
pub = rospy.Publisher("robot", Robot, queue_size=10)


def main():
    rospy.init_node("spawner")

    print(rospy.get_param_names())

    robot_id = rospy.get_param("~id", None)
    position = Vector3(x=rospy.get_param("~x", None), y=rospy.get_param(
        "~y", None), z=rospy.get_param("~z", None))
    rotation = rospy.get_param("~rotation", None)

    global robot
    robot = Robot(robot_id=robot_id, position=position, rotation=rotation)

    rospy.Subscriber("presence", Int32, on_presence)

    rospy.spin()


def on_presence(data):
    pub.publish(robot)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
