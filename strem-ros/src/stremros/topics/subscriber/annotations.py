import rospy

class AnnotationSubscriber:
    """A ROS subscriber for object annotations data.
    """

    def __init__(self):
        rospy.init_node("annotation", anonymous=False)
        rospy.Subscriber("carla/objects", )

    def callback(self, msg):
        """A handler for ROS messsages.
        """

        print(msg)
    
    def listen(self):
        rospy.spin()
