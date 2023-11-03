
class RobotNavigator:

    def __init__(self, robot):
        self.robot = robot


    def navigate(self):
        self.path_planner.plan_path()
        action = self.path_planner.get_next_action()

        while action:
            if action == "move_forward":
                # Use o Lidar para verificar se há um obstáculo à frente.
                # Se houver um obstáculo (por exemplo, uma parede), pare.
                if self.robot.lidar_detected_wall():
                    self.robot.stop()
                else:
                    self.robot.move_forward()
            elif action == "turn_left":
                self.robot.turn_by_angle(-90)
            elif action == "turn_right":
                self.robot.turn_by_angle(90)
            elif action == "stop":
                self.robot.stop()
            
            action = self.path_planner.get_next_action()
