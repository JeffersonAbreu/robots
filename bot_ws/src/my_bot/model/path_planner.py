class PathPlanner:

    def __init__(self):
        self.path = []

    def plan_path(self):
        # Aqui, você pode definir uma sequência de ações básicas para o robô seguir.
        # Por exemplo: ["move_forward", "turn_left", "move_forward", "turn_right", "stop"]
        self.path = ["move_forward", "turn_left", "move_forward", "turn_right", "move_forward", "stop"]

    def get_next_action(self):
        if self.path:
            return self.path.pop(0)
        return None
