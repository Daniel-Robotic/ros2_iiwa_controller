import os
import json
import rclpy
import numpy as np
from pathlib import Path

# Импорты ROS2 и сообщений
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration as MsgDuration
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Импорт пользовательских классов робота и планировщика движения
from motion_planner import KukaIiwaRobot, IiwaMotionPlanner


class JsonCommandExecutor(Node):
    def __init__(self):
        super().__init__('json_command_executor')  # Инициализация ROS2-ноды

        # Путь к JSON-файлу по умолчанию (внутри пакета iiwa_description)
        json_path = os.path.join(get_package_share_directory("iiwa_description"), 
                                "config", "robot_poses.json")

        # Объявление параметров: путь к json, шаг дискретизации, имя топика
        self.declare_parameter('json_path', json_path)
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('trajectory_topic', '/iiwa_arm_controller/joint_trajectory')

        # Имена суставов манипулятора
        self.joint_names = [
            'lbr_iiwa7_A1', 'lbr_iiwa7_A2', 'lbr_iiwa7_A3',
            'lbr_iiwa7_A4', 'lbr_iiwa7_A5', 'lbr_iiwa7_A6', 'lbr_iiwa7_A7'
        ]

        # Получение значений параметров
        self.json_path = Path(self.get_parameter('json_path').get_parameter_value().string_value)
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value

        # Инициализация модели робота и планировщика
        self.robot = KukaIiwaRobot()
        self.planner = IiwaMotionPlanner(self.robot, dt=self.dt)  # type: ignore
        self.q_actual = self.robot.qz  # Начальная конфигурация (например, нулевая)
        self.planner.updateCurrentPos(self.q_actual)  # type: ignore

        # Подписка на топик с текущими состояниями суставов
        self.create_subscription(JointState, '/joint_states', self._cb_joint_state, 10)

        # Паблишер траектории
        self.traj_pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)

        # Клиент к сервису камеры (Trigger) (Меняем на свой)
        self.cam_cli = self.create_client(Trigger, '/camera/trigger')

        # Загрузка команд из JSON
        try:
            with open(self.json_path, 'r', encoding='utf-8') as f:
                self.commands = json.load(f)
                self.get_logger().info(f"Загружено {len(self.commands)} команд из {self.json_path}")
        except Exception as e:
            self.get_logger().error(f"Не удалось загрузить JSON: {e}")
            self.commands = []

        # Внутренние переменные для отслеживания состояния
        self._collecting = False             # Находится ли сейчас в collect-зоне
        self._cam_triggered = False          # Была ли уже вызвана камера
        self._collect_windows = []           # Временные окна для съёмки
        self._trajectory = []                # Траектория как список конфигураций
        self._timepoints = []                # Временные метки
        self._trajectory_interp = {}         # Интерполированная траектория (t -> q)
        self._t = 0.0                        # Текущее ROS-время от старта
        self._start_time_ros = None          # Время старта траектории в ROS

        self._build_full_trajectory()
        # Таймер для автоматической FSM-логики
        self.create_timer(0.1, self._fsm_step)

    def _cb_joint_state(self, msg: JointState):
        """
        Колбэк при получении новых значений состояний суставов
        """
        # Сопоставление имён суставов и значений
        pos_map = dict(zip(msg.name, msg.position))
        try:
            # Обновляем актуальную позу
            self.q_actual = np.array([pos_map[name] for name in self.joint_names])
            self.planner.updateCurrentPos(self.q_actual)  # type: ignore

            # Проверяем, достигнута ли целевая точка (для съёмки)
            if self._collecting and not self._cam_triggered and self._start_time_ros is not None:
                now_ros = self.get_clock().now().nanoseconds * 1e-9
                elapsed = now_ros - self._start_time_ros

                if self._trajectory_interp:
                    # Находим ближайшую по времени точку
                    nearest_t = min(self._trajectory_interp.keys(), key=lambda t: abs(t - elapsed))
                    target_q = self._trajectory_interp[nearest_t]

                    # Проверка на точность достижения
                    err = np.max(np.abs(self.q_actual - target_q))
                    self.get_logger().info(f"Отклонение от текущей целевой позы: {err:.4f}")

                    if err <= 0.05:
                        self._trigger_camera()
                        self._cam_triggered = True
                        
        except KeyError as e:
            self.get_logger().warning(f"Некорректные имена суставов: {e}")

    def _fsm_step(self):
        """
        Логика FSM (Finite State Machine) – проверка входа и выхода из collect-зон
        """
        now_ros_time = self.get_clock().now().nanoseconds * 1e-9
        if self._start_time_ros is None:
            return

        elapsed_time = now_ros_time - self._start_time_ros
        self.get_logger().debug(f"[FSM] Elapsed: {elapsed_time:.2f}")
        self.get_logger().debug(f"[FSM] Окна collect: {self._collect_windows}")

        active = False
        for t_start, t_end in self._collect_windows:
            if t_start <= elapsed_time <= t_end:
                if not self._collecting:
                    self.get_logger().info(f"▶️ Вошли в collect-зону ({t_start:.2f}–{t_end:.2f})")
                self._collecting = True
                active = True
                break

        if not active and self._collecting:
            self._collecting = False
            self._cam_triggered = False
            self.get_logger().info("⏹ Выход из collect-зоны")

    def _build_full_trajectory(self):
        """
        Построение полной траектории движения на основе JSON-команд
        """
        self.get_logger().info("Формируем полную траекторию...")
        current_q = self.q_actual.copy()

        # Обнуление всех предыдущих данных
        self._trajectory = []
        self._timepoints = []
        self._collect_windows = []
        self._trajectory_interp = {}
        self._t = 0.0

        # Вспомогательная функция для добавления траектории
        def append_traj(q_mat):
            for q in q_mat:
                self._trajectory.append(q)
                self._timepoints.append(self._t)
                self._trajectory_interp[self._t] = q
                self._t += self.dt

        # Движение в начальную позицию (qr)
        traj0 = self.planner.move('joints', self.robot.qr.tolist(), velocity=0.1)
        append_traj(traj0)
        current_q = traj0[-1]

        # Выполнение команд из JSON
        for i, cmd in enumerate(self.commands):
            if cmd['command'] == 'move':
                self.get_logger().info(f"[{i}] move: {np.round(cmd['args'], 3)}")
                self.planner.updateCurrentPos(current_q)
                traj = self.planner.move('ptp', cmd['args'], velocity=0.1)
                append_traj(traj)
                current_q = traj[-1]
            elif cmd['command'] == 'collect':
                self.get_logger().info(f"[{i}] collect: удержание на 3 с")
                t_start = self._t
                steps = int(3.0 / self.dt)
                hold = np.repeat(current_q[None, :], steps, axis=0)
                append_traj(hold)
                t_end = self._t
                self._collect_windows.append((t_start, t_end))

        self.get_logger().info(f"Итоговая длина траектории: {len(self._trajectory)} точек")
        self._send_full_trajectory()

    def _send_full_trajectory(self):
        """
        Отправка собранной траектории в топик контроллера
        """
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for q, t in zip(self._trajectory, self._timepoints):
            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            pt.time_from_start = MsgDuration()
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1.0) * 1e9)
            traj.points.append(pt)  # type: ignore

        self._start_time_ros = self.get_clock().now().nanoseconds * 1e-9
        self.traj_pub.publish(traj)

        self.get_logger().info(f"✅ Опубликована общая траектория из {len(traj.points)} точек")
        self.get_logger().info(f"🕒 Траектория стартовала в ROS-времени {self._start_time_ros:.2f}")

    def _trigger_camera(self):
        """
        Асинхронный вызов ROS-сервиса для съёмки изображения
        """
        self.get_logger().info("Проверка доступности сервиса камеры...")
        if not self.cam_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("Сервис камеры недоступен – пропускаем съёмку")
            return

        future = self.cam_cli.call_async(Trigger.Request())

        def cb(fut):
            try:
                res = fut.result()
                self.get_logger().info(f"📸 Камера: {res.success}, сообщение: {res.message}")
            except Exception as e:
                self.get_logger().error(f"Ошибка при вызове сервиса камеры: {e}")

        future.add_done_callback(cb)


def main():
    rclpy.init()
    executor = JsonCommandExecutor()
    # executor._build_full_trajectory()
    rclpy.spin(executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
