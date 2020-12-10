import rospy

class service(object):
    def __init__(self, path, srv_type):
        self.path = path
        self.type = srv_type
        rospy.loginfo("Waiting for service %s" %self.path)
        rospy.wait_for_service(self.path)
        self.srv = rospy.ServiceProxy(self.path, self.type)
        rospy.loginfo("Got service %s" %self.path)
