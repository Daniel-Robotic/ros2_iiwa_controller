import os
import rclpy
import pandas as pd

from pathlib import Path
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CSVTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('csv_trajectory_publisher')

        # Путь к CSV-файлу по умолчанию (внутри пакета iiwa_description)
        csv_path = os.path.join(get_package_share_directory("iiwa_description"), 
                                "config", "example_move.csv")

        # Объявление параметров ROS2: путь к CSV, шаг дискретизации и имя топика
        self.declare_parameter('csv_path', csv_path)
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('topic', '/iiwa_arm_controller/joint_trajectory')

        # Получение значений параметров
        csv_path = Path(self.get_parameter('csv_path').get_parameter_value().string_value)
        self.dt = float(self.get_parameter('dt').get_parameter_value().double_value)
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # Имена суставов манипулятора (в порядке от основания к фланцу)
        self.joint_names = [
            'lbr_iiwa7_A1', 'lbr_iiwa7_A2', 'lbr_iiwa7_A3',
            'lbr_iiwa7_A4', 'lbr_iiwa7_A5', 'lbr_iiwa7_A6', 'lbr_iiwa7_A7'
        ]

        # Попытка считать CSV-файл с углами суставов
        try:
            df = pd.read_csv(csv_path, index_col=False)
        except Exception as e:
            self.get_logger().fatal(f'Не удалось открыть CSV «{csv_path}»: {e}')
            rclpy.shutdown()
            return

        # Проверка: в файле должно быть ровно 7 столбцов (по числу суставов)
        if df.shape[1] != 7:
            self.get_logger().fatal(f'CSV должен содержать 7 столбцов, найдено {df.shape[1]}')
            rclpy.shutdown()
            return

        # Конвертация содержимого CSV в numpy-массив
        self.q_matrix = df.to_numpy()

        # Проверка: должно быть хотя бы две точки, чтобы траектория имела смысл
        if len(self.q_matrix) < 2:
            self.get_logger().fatal('В CSV меньше двух точек – траектория не имеет смысла')
            rclpy.shutdown()
            return

        self.get_logger().info(f'CSV прочитан: {len(self.q_matrix)} точек, dt={self.dt}s')

        # Создание паблишера для публикации сообщений типа JointTrajectory
        self.pub = self.create_publisher(JointTrajectory, topic, 10)

        # Создание таймера: через 0.5 секунды будет вызвана функция публикации
        self.create_timer(0.5, self._publish_once)

        # Флаг, чтобы отправка происходила только один раз
        self.sent = False

    def _publish_once(self):
        if self.sent:
            return  # Если уже отправлено – ничего не делаем

        traj = JointTrajectory()
        traj.joint_names = self.joint_names  # Указание имён суставов

        # Формируем список точек траектории
        for i, q in enumerate(self.q_matrix):
            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()  # Задание позиций для всех 7 суставов

            # Расчёт времени от начала траектории
            t = i * self.dt
            pt.time_from_start = Duration(
                sec=int(t),                         # Целые секунды
                nanosec=int((t % 1.0) * 1e9)        # Остаток – в наносекундах
            )
            # Добавление точки в список
            traj.points.append(pt)  # type: ignore 

        # Публикация сформированной траектории
        self.pub.publish(traj)
        self.get_logger().info(f'Отправлено {len(traj.points)} точек в {self.pub.topic}')
        self.sent = True  # Флаг, чтобы не отправлять повторно


def main():
    rclpy.init()
    node = CSVTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
