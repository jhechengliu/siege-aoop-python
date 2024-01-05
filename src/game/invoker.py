import warnings

class Invoker():
    """
    Invoker class in the Command Pattern.

    The Invoker is responsible for managing and executing commands. It acts as an intermediary
    between the client and the actual command objects. It encapsulates the execution of commands,
    allowing for parameterization, queuing, and logging.

    Attributes:
        __instance (Invoker): The instance of the Invoker class.
        __commands (list): A list to store the history of executed commands.
        last_command (Command): The last executed command.

    Methods:
        get_instance(): Returns the instance of the Invoker class.
        execute(command): Execute the given command.
        undo(): Undo the last executed command.
        batch_execute(commands): Execute a batch of commands simultaneously.
    """
    __instance = None

    def __init__(self) -> None:

        """
        Initialize the Invoker with an empty list to store commands.
        """
        self.__commands = []
        warnings.warn("Use get_instance class method to obtain the instance", UserWarning)

    @classmethod
    def get_instance():
        if Invoker.__instance == None:
            Invoker.__instance = Invoker()
        
        return Invoker.__instance

    def execute(self, command):
        """
        Execute the given command and add it to the command history.

        Args:
            command: The command to be executed.
        """
        command.execute()
        self.__commands.append(command)

    def undo(self):
        """
        Undo the last executed command.

        Raises:
            IndexError: If there are no commands to undo.
        """
        try:
            last__command = self.__commands.pop()
            last__command.undo()
        except IndexError:
            raise IndexError("No commands to undo.")

    def batch_execute(self, commands):
        """
        Execute a batch of commands simultaneously.

        Args:
            commands (list): A list of commands to be executed.
        """
        for command in commands:
            self.execute(command)
