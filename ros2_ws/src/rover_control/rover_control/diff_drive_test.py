import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys

class RoverController(Node):

    def __init__(self, state_times):
        super().__init__('rover_controller')

        self.left_wheel_publisher = self.create_publisher(Twist, '/diff_drive_controller_left/cmd_vel_unstamped', 10)
        self.right_wheel_publisher = self.create_publisher(Twist, '/diff_drive_controller_right/cmd_vel_unstamped', 10)

        self.state = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Kąt orientacji
        self.d = 0.037  # Rozstaw kół kalibracja

        # Subskrypcja tematów odometrii
        self.left_odom_subscriber = self.create_subscription(
            Odometry,
            '/diff_drive_controller_left/odom',
            self.left_odom_callback,
            10
        )
        self.right_odom_subscriber = self.create_subscription(
            Odometry,
            '/diff_drive_controller_right/odom',
            self.right_odom_callback,
            10
        )

        self.left_linear_velocity = 0.0
        self.right_linear_velocity = 0.0

        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        self.state_times = state_times  # Użycie przekazanych argumentów
        self.state_end_time = [self.state_start_time + self.state_times[0], 
                               self.state_start_time + self.state_times[0] + self.state_times[1], 
                               self.state_start_time + self.state_times[0] + self.state_times[1] + self.state_times[2], 
                               self.state_start_time + self.state_times[0] + self.state_times[1] + self.state_times[2] + self.state_times[3], 
                               0.0]  # Czas zakończenia dla każdego etapu

        self.timer_period = 0.1  # Interwał aktualizacji
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def left_odom_callback(self, msg):
        """Callback dla odometrii lewego koła"""
        self.left_linear_velocity = msg.twist.twist.linear.x

    def right_odom_callback(self, msg):
        """Callback dla odometrii prawego koła"""
        self.right_linear_velocity = msg.twist.twist.linear.x

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        left_twist = Twist()
        right_twist = Twist()

        # W zależności od stanu, ustawiamy prędkości kół
        if current_time < self.state_end_time[0]:  # Jazda w lewo przez 3 sekundy
            left_twist.linear.x = 2.0  # Ruch w lewo (lewą stroną do przodu)
            right_twist.linear.x = -2.0  # Ruch w lewo (prawą stroną do tyłu)
            self.get_logger().info("State 0: Moving left")
        elif current_time < self.state_end_time[1]:  # Jazda w prawo przez 2 sekundy
            left_twist.linear.x = -2.0  # Ruch w prawo (lewą stroną do tyłu)
            right_twist.linear.x = 2.0  # Ruch w prawo (prawą stroną do przodu)
            self.get_logger().info("State 1: Moving right")
        elif current_time < self.state_end_time[2]:  # Jazda w prawo przez 2 sekundy
            left_twist.linear.x = 2.0  # Ruch w prawo (lewą stroną do tyłu)
            right_twist.linear.x = 2.0  # Ruch w prawo (prawą stroną do przodu)
            self.get_logger().info("State 1: Moving right")
        elif current_time < self.state_end_time[3]:  # Jazda w prawo przez 2 sekundy
            left_twist.linear.x = -2.0  # Ruch w prawo (lewą stroną do tyłu)
            right_twist.linear.x = -2.0  # Ruch w prawo (prawą stroną do przodu)
            self.get_logger().info("State 1: Moving right")
        else:  # Zatrzymanie
            left_twist.linear.x = 0.0
            right_twist.linear.x = 0.0
            self.get_logger().info("State 2: Stopped")

        # Publikujemy prędkości kół
        self.left_wheel_publisher.publish(left_twist)
        self.right_wheel_publisher.publish(right_twist)

        # Zmienność prędkości kół na podstawie odometrii
        v = (self.left_linear_velocity + self.right_linear_velocity) / 2.0
        omega = (self.right_linear_velocity - self.left_linear_velocity) / self.d

        # Kinematyka pojazdu różnicowego (obliczanie zmiany pozycji i kąta)
        delta_t = self.timer_period  # Czas krokowy
        self.x += v * math.cos(self.theta) * delta_t
        self.y += v * math.sin(self.theta) * delta_t
        self.theta += omega * delta_t

        # Ustalamy końcowy kąt
        self.get_logger().info(f"Position: x = {self.x:.2f}, y = {self.y:.2f}, theta = {self.theta:.2f}")

        # Po zakończeniu jazdy, zatrzymanie i anulowanie timera
        if current_time >= self.state_end_time[3]:
            self.state_end_time[4] = current_time
            self.timer.cancel()  # Zatrzymujemy timer, bo cała sekwencja się zakończyła

def main(args=None):
    rclpy.init(args=args)

    # Przetwarzanie argumentów z wiersza poleceń
    if len(sys.argv) != 5:
        print("Usage: ros2 run rover_control diff_test <time_left> <time_right>")
        sys.exit(1)
    
    try:
        time_left = float(sys.argv[1])
        time_right = float(sys.argv[2])
        time_forward = float(sys.argv[3])
        time_backward = float(sys.argv[4])
    except ValueError:
        print("Error: Time values must be numeric.")
        sys.exit(1)

    # Tworzenie instancji kontrolera z przekazanymi czasami
    rover_controller = RoverController(state_times=[time_left, time_right, time_forward, time_backward, 0.0])

    rclpy.spin(rover_controller)

    rover_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
