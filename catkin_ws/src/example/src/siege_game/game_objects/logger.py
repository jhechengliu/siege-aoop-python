import rospy

class Logger():
    def __init__(self, name):
        self.__name = name

    def info(self, msg, *args, **kwargs):
        rospy.loginfo(f"[{self.__name}]", msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        rospy.logerr(f"[{self.__name}]", msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        rospy.logdebug(f"[{self.__name}]", msg, *args, **kwargs)

    def warn(self, msg, *args, **kwargs):
        rospy.logwarn(f"[{self.__name}]", msg, *args, **kwargs)

    def fatal(self, msg, *args, **kwargs):
        rospy.logfatal(f"[{self.__name}]", msg, *args, **kwargs)