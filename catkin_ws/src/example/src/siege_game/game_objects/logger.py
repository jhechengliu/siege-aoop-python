import rospy

class Logger():
    def __init__(self, name):
        self.__name = f"[{name}]"

    def info(self, msg):
        rospy.loginfo(self.__name + " " + msg)

    def error(self, msg):
        rospy.logerr(self.__name + " "  + msg)

    def debug(self, msg):
        rospy.logdebug(self.__name + " "  + msg)

    def warn(self, msg):
        rospy.logwarn(self.__name + " "  + msg)

    def fatal(self, msg):
        rospy.logfatal(self.__name + " "  + msg)