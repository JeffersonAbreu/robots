from enum import Enum, auto

class CommandType(Enum):
    MOVE_FORWARD = auto()
    MOVE_BACKWARD = auto()
    TURN = auto()
    STOP = auto()
    START_CURVE = auto()

class Command:
    def __init__(self, command_type: CommandType, value=None):
        self.type = command_type
        self.value = value

class CommandQueue:
    def __init__(self):
        self.queue = []

    def add_command(self, command: Command):
        """Add a command to the queue."""
        self.queue.append(command)

    def get_next_command(self):
        if self.queue:
            return self.queue.pop(0)

    def clear(self):
        self.queue.clear()
