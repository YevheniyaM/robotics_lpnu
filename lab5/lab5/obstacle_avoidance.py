"""
Оминання перешкод (Obstacle Avoidance)

Реалізація логіки руху до цілі з урахуванням перешкод.
Використовує /scan (лазерний сканер) та /odom (одометрія); публікує команди у /cmd_vel.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        # Дозволяємо вузлу використовувати час симуляції Gazebo (важливо для синхронізації)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Налаштування QoS (Quality of Service). 
        # BEST_EFFORT дозволяє не чекати підтвердження отримання кожного пакету даних від лідара.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Параметри цілі та порогів спрацювання
        self.goal_x = self.declare_parameter("goal_x", 3.0).value
        self.goal_y = self.declare_parameter("goal_y", 3.0).value
        self.dist_threshold = 0.5   # Мінімальна відстань до перешкоди (метрів)
        self.goal_tolerance = 0.1   # Радіус "попадання" у ціль (метрів)

        # Внутрішній стан робота (координати та кут повороту)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0      # Напрямок, куди дивиться "обличчя" робота
        self.obstacle_ahead = False # Прапорець: чи бачимо стіну попереду

        # Підписки на топіки (використовуємо налаштований qos)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, qos)
        
        # Паблішер для відправки команд швидкості (формат TwistStamped)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # Таймер керування: запускає функцію control_loop 10 разів на секунду (10 Гц)
        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        """ Обробка даних з лазерного сканера (LiDAR) """
        # Беремо сектор +/- 30 градусів перед роботом
        # Масив ranges зазвичай починається з 0 (центр) і йде по колу
        front_ranges = msg.ranges[:30] + msg.ranges[-30:]
        
        # Відфільтровуємо некоректні значення (занадто близькі або нескінченність)
        valid_ranges = [r for r in front_ranges if r > 0.05 and not math.isinf(r)]
        
        if valid_ranges:
            min_dist = min(valid_ranges)
            # Якщо найближчий об'єкт ближче порогу — активуємо зупинку
            self.obstacle_ahead = min_dist < self.dist_threshold
        else:
            self.obstacle_ahead = False

    def odom_callback(self, msg):
        """ Отримання поточної позиції робота з одометрії """
        # Отримуємо X та Y
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y

        # Перетворюємо орієнтацію з кватерніона (x, y, z, w) у кут Ейлера (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """ Основний цикл прийняття рішень """
        # Виводимо координати в консоль раз на 2 секунди, щоб не забивати лог
        self.get_logger().info(
            f"Позиція: ({self.current_x:.1f}, {self.current_y:.1f}), Перешкода: {self.obstacle_ahead}", 
            throttle_duration_sec=2.0
        )

        # Створюємо об'єкт повідомлення швидкості
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Рахуємо пряму відстань до цілі за формулою Піфагора
        dist_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

        # Перевірка: чи ми вже на місці?
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info("!!! ЦІЛЬ ДОСЯГНУТА !!!")
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return

        # Обчислюємо необхідний кут, щоб повернутися обличчям до цілі
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        # Різниця між потрібним кутом і тим, куди ми зараз дивимось
        angle_diff = angle_to_goal - self.current_yaw
        
        # Нормалізація: робимо так, щоб кут завжди був у межах [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # --- ЛОГІКА РУХУ ---
        if self.obstacle_ahead:
            # Якщо попереду стіна: зупиняємось і крутимось на місці, шукаючи вихід
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.5
        else:
            # Якщо шлях вільний:
            if abs(angle_diff) > 0.3:  
                # Якщо ми сильно відхилилися від курсу — повертаємо активніше (майже на місці)
                msg.twist.linear.x = 0.05
                msg.twist.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                # Якщо дивимось приблизно на ціль — їдемо швидко і плавно підрулюємо
                msg.twist.linear.x = 0.2
                msg.twist.angular.z = angle_diff * 0.5 

        # Відправляємо команду моторам
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Примусова зупинка робота перед вимкненням скрипта
        stop_msg = TwistStamped()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()