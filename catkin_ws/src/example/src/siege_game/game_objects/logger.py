import rospy

class Logger():
    """
    The Logger class provides logging functionality for the Siege game.

    Attributes:
        __name: The name of the logger.

    Methods:
        info(msg): Logs an informational message.
        warning(msg): Logs a warning message.
        error(msg): Logs an error message.
        fatal(msg): Logs a fatal error message.
        debug(msg): Logs a debug message.
    """
    def __init__(self, name):
        """
        Initializes a new instance of the Logger class.

        Args:
            name (str): The name of the logger.
        """
        self.__name = f"[{name}]"

    def info(self, msg):
        """
        Logs an informational message.

        Args:
            msg: The message to log.
        """
        rospy.loginfo(self.__name + " " + msg)

    def error(self, msg):
        """
        Logs an error message.

        Args:
            msg: The message to log.
        """
        rospy.logerr(self.__name + " "  + msg)

    def debug(self, msg):
        """
        Logs a debug message.

        Args:
            msg: The message to log.
        """
        rospy.logdebug(self.__name + " "  + msg)

    def warning(self, msg):
        """
        Logs a warning message.

        Args:
            msg: The message to log.
        """
        rospy.logwarn(self.__name + " "  + msg)

    def fatal(self, msg):
        """
        Logs a fatal error message.

        Args:
            msg: The message to log.
        """
        rospy.logfatal(self.__name + " "  + msg)