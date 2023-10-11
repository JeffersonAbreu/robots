from geometry_msgs.msg import Twist
from utils.state import State
from utils import RobotEstimator
from utils.bib import degrees_to_radians

class Robot:

    def __init__(self, node):
        self.node = node
        self.robot_estimator = RobotEstimator(node)
        self.turn_timer = None
        self.state = State.STOP
        self.old_state = State.STOP
        self.current_orientation = 0.0
        self.destiny_orientation = 0.0
        self.initial_orientation = 0.0

        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)


    def __turn_timer_callback(self):
        self.current_orientation = self.robot_estimator.get_orientation()
        difference = 0.0
        angular_speed = 0.0
        if self.state == State.TURN:
            if abs(self.current_orientation - self.destiny_orientation) <= 0.001:
                self.stop()
                self.state = State.MOVING_FORWARD
                self.turn_timer.cancel()
                self.node.get_logger().warn(self.state.name)
                self.move_forward()
            else:
                difference = self.destiny_orientation - self.current_orientation
                # Limitar a velocidade angular para evitar viradas muito rápidas
                angular_speed = max(min(difference, 2.0), -2.0)
                diff = abs(angular_speed)
                if diff < 0.05:
                    if diff > 0.00:
                        angular_speed =  angular_speed * 0.75 # speed: 75%
                    else:
                        if diff > 0.005:
                            angular_speed = angular_speed * 0.5 # speed: 50%
                        else:
                            if diff > 0.0005:
                                angular_speed = angular_speed * 0.25 # speed: 25%
                            else:
                                angular_speed = angular_speed * 0.1 # speed: 10%
        
                if angular_speed > 0:
                    self.__turn_speed_right(angular_speed)    
                else: 
                    self.__turn_speed_left(-angular_speed)    

                self.status(difference, angular_speed, diff)


    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.3
        self.twist_pub.publish(twist)
        self.state = State.FORWARD


    def stop(self):
        twist = Twist()
        self.twist_pub.publish(twist)
        self.state = State.STOP
    

    def __turn_speed_left(self, angle_speed):
        self.__turn_speed(angle_speed * -1)
    

    def __turn_speed_right(self, angle_speed):
        self.__turn_speed(angle_speed)
        

    def __turn_speed(self, angle_speed):
        twist = Twist()
        twist.angular.z = angle_speed
        self.twist_pub.publish(twist)


    def turn_by_angle(self, angle):
        self.state = State.TURN
        self.initial_orientation = self.current_orientation = self.robot_estimator.get_orientation()
        self.destiny_orientation = self.current_orientation + degrees_to_radians(angle)
        self.turn_timer = self.node.create_timer(0.1, self.__turn_timer_callback)  # Verifica a cada 0.1 segundos



    def status(self, difference, angular_speed, diff):
        texto = f"Destiny: {self.destiny_orientation:.6f}   Current: {self.current_orientation:.6f}   Difference: {difference:.6f}   angular_speed: {angular_speed:.6f} diff: {diff}"
        self.node.get_logger().info(texto)

    def get_state(self):
        return self.state

