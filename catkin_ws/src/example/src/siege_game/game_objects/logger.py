import rospy

class Logger():
    def __init__(self, name):
        self.__name = f"[{name}]"

    def info(self, msg, *args, **kwargs):
        rospy.loginfo(self.__name, msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        rospy.logerr(self.__name, msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        rospy.logdebug(self.__name, msg, *args, **kwargs)

    def warn(self, msg, *args, **kwargs):
        rospy.logwarn(self.__name, msg, *args, **kwargs)

    def fatal(self, msg, *args, **kwargs):
        rospy.logfatal(self.__name, msg, *args, **kwargs)