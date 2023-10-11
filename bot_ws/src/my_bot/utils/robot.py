from geometry_msgs.msg import Twist
from utils.state import State
from utils import RobotEstimator
from utils.bib import degrees_to_radians
from rclpy.clock import Clock, ClockType
import threading

class Robot:

    def __init__(self, node):
        self.node = node
        Hz = 30
        self.robot_estimator = RobotEstimator(node)
        #self.timer = self.node.create_timer(Hz/100, self.timer_callback)

        self.state = State.STOP
        self.old_state = State.STOP
        self.estimate_linear = 0.0
        self.estimate_angular = 0.0
        self.clock = Clock(clock_type=ClockType.ROS_TIME)

        self.twist_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)


    def timer_callback(self):
        self.estimate_linear  = self.robot_estimator.get_velocity()
        self.estimate_angular = self.robot_estimator.get_orientation()


    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.3
        self.twist_pub.publish(twist)
        self.state = State.FORWARD
        self.log_state()


    def stop(self):
        twist = Twist()
        self.twist_pub.publish(twist)
        self.state = State.STOP
        self.log_state()
    

    def __turn_speed_left(self, angle_speed):
        self.__turn_speed(angle_speed * -1)
    

    def __turn_speed_right(self, angle_speed):
        self.__turn_speed(angle_speed)
        

    def __turn_speed(self, angle_speed):
        twist = Twist()
        twist.angular.z = angle_speed
        self.twist_pub.publish(twist)
        self.log_state()  


    def turn_by_angle(self, angle):
      turn_thread = threading.Thread(target=self.__turn_by_angle_thread, args=(angle,))
      turn_thread.start()
      
    def __turn_by_angle_thread(self, angle):
      self.state = State.TURN
      self.timer_callback()
      current_orientation = self.robot_estimator.get_orientation()
      initial_orientation = current_orientation
      target_orientation = current_orientation + degrees_to_radians(angle)
      self.status(initial_orientation, current_orientation, target_orientation, target_orientation - current_orientation)
      start_time = self.clock.now() # Registra o tempo inicial
      timeout = 5  # 5 segundos

      while abs(current_orientation - target_orientation) > 0.05:  # 0.05 é uma tolerância em radianos
          elapsed_time = (self.clock.now() - start_time).nanoseconds / 1e9
          if elapsed_time > timeout:
            self.node.get_logger().warn("Timeout reached while turning!")
            break
          
          difference = target_orientation - current_orientation
          angular_speed = 1.0 * difference  # 1.0 é um ganho proporcional arbitrário

          # Limitar a velocidade angular para evitar viradas muito rápidas
          angular_speed = max(min(angular_speed, 1.0), -1.0)

          if angular_speed > 0:
              self.__turn_speed_right(angular_speed)
          else:
              self.__turn_speed_left(-angular_speed)
          #self.status(initial_orientation, current_orientation, target_orientation, difference)
          self.robot_estimator.print_orientation()
          current_orientation = self.robot_estimator.get_orientation()

      self.stop()

    def status(self, inicial, atual, destino, difference):
        space = "                                                     "
        texto = f"   Inicial: {inicial:.6f}\n{space}   Destiny: {destino:.6f}\n{space}   Current: {atual:.6f}\n{space}Difference: {difference:.6f}"
        self.node.get_logger().info(texto)

    def get_state(self):
        return self.state
    
    def log_state(self):
        #self.node.get_logger().info(f'Robot position: {self.state.name}')
        pass

