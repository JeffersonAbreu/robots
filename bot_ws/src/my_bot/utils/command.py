from enum import Enum, auto

class CommandType(Enum):
    MOVE_FORWARD = auto()
    MOVE_BACKWARD = auto()
    TURN = auto()
    STOP = auto()
    CURVE = auto()

class Command:
    def __init__(self, command_type: CommandType, value=None):
        self.type = command_type
        self.value = value

class CommandQueue:
    '''
    FilaDeComandos
    '''
    def __init__(self):
        self.queue = []

    def add_command(self, command: Command):
        """Adiciona o comando a lista."""
        self.queue.append(command)

    def add_command_to_init(self, command: Command):
        """Adiciona o comando ao inicio da lista."""
        self.queue.insert(0, command)

    def get_next_command(self) -> Command:
        if self.queue:
            return self.queue.pop(0)

    def clear(self):
        self.queue.clear()

    def size(self):
        return len(self.queue)
