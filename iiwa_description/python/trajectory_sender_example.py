import rclpy 
import numpy as np

from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.callback_groups import ReentrantCallbackGroup
from iiwa_interfaces.srv import ExecuteJointGoal, ExecutePoseGoal

from motion_planner import IiwaMotionPlanner, KukaIiwaRobot


class JointTrajectoryServer(Node):
    """
    ROS2-нода для управления движением робота KUKA iiwa 7 через action-интерфейс FollowJointTrajectory.
    Поддерживает движение в суставное пространство, в позицию ЭЭ, а также команды "home", "work" и плавную остановку.
    """

    DT = 0.01  # Шаг дискретизации траектории (10 мс)

    def __init__(self):
        super().__init__('joint_trajectory_server')

        # 1. Имена суставов робота
        self.joint_names = [
            'lbr_iiwa7_A1','lbr_iiwa7_A2','lbr_iiwa7_A3',
            'lbr_iiwa7_A4','lbr_iiwa7_A5','lbr_iiwa7_A6','lbr_iiwa7_A7'
        ]

        # 2. Инициализация модели робота и планировщика
        self.planner = IiwaMotionPlanner(KukaIiwaRobot(), dt=self.DT)  # type: ignore

        # 3. Инициализация action-клиента к контроллеру
        self.group  = ReentrantCallbackGroup()
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/iiwa_arm_controller/follow_joint_trajectory',
                                   callback_group=self.group)
        self.client.wait_for_server()

        # 4. Попытка отменить все цели после запуска
        try:
            self.client._client._cancel_all_goals()  # type: ignore
        except Exception:
            pass

        # 5. Внутренние переменные для отслеживания состояния
        self.q_actual = self.planner.robot.qz  # type: ignore
        self.goal_handle = None
        self.q_goal_sent = None
        self._q_start = None
        self._q_goal = None

        # 6. Подписка на состояния суставов
        self.create_subscription(JointState,'/joint_states', self._cb_joint_state,10)

        # 7. Публикация позы ЭЭ в виде (x, y, z, roll, pitch, yaw)
        self.ee_state_pub = self.create_publisher(Float64MultiArray, '/robot_pose_ee', 10)

        # 8. Сервисы управления движением
        self.create_service(ExecuteJointGoal, 'execute_joint_goal', self._srv_joint, callback_group=self.group)
        self.create_service(ExecutePoseGoal, 'execute_pose_goal', self._srv_pose, callback_group=self.group)
        self.create_service(Trigger, 'cancel_trajectory', self._srv_cancel, callback_group=self.group)
        self.create_service(Trigger, 'execute_home_pose', self._srv_home, callback_group=self.group)
        self.create_service(Trigger, 'execute_work_pose', self._srv_work, callback_group=self.group)

    def _send_to_controller(self, q_matrix: np.ndarray):
        """
        Формирует и отправляет траекторию в контроллер. 
        Начальная точка — текущая поза. Далее — путь q_matrix.
        """
        if self.goal_handle and self.goal_handle.status in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING):
            self.get_logger().warn('Контроллер занят — цель отклонена')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        # 1. Добавляем первую точку — текущую конфигурацию
        pt0 = JointTrajectoryPoint()
        pt0.positions = self.q_actual.tolist()
        pt0.time_from_start = Duration()
        goal.trajectory.points.append(pt0)  # type: ignore

        # 2. Добавляем точки из планируемой траектории
        for k, q in enumerate(q_matrix, start=1):
            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            t = k * self.DT
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)  # type: ignore

        # 3. Сохраняем траекторию для отслеживания прогресса
        self._q_start = self.q_actual.copy()
        self._q_goal = q_matrix[-1].copy()
        self.q_goal_sent = self._q_goal.copy()

        fut = self.client.send_goal_async(goal, feedback_callback=self._cb_fb)
        fut.add_done_callback(self._cb_goal_resp)
        return True

    def _cb_joint_state(self, msg: JointState):
        """
        Обновляет текущее состояние суставов и публикует позу ЭЭ.
        """
        mp = dict(zip(msg.name, msg.position))
        try:
            self.q_actual = np.array([mp[n] for n in self.joint_names])
            self.planner.updateCurrentPos(self.q_actual)  # type: ignore

            T = self.planner.robot.fkine(self.q_actual)  # type: ignore
            xyz = T.t
            rpy = T.rpy(order='zyx', unit='rad')

            msg = Float64MultiArray() # type: ignore
            msg.data = [*xyz.tolist(), *rpy.tolist()] # type: ignore
            self.ee_state_pub.publish(msg)
        except KeyError:
            pass

    def _srv_joint(self, req: ExecuteJointGoal.Request, res: ExecuteJointGoal.Response):
        """
        Сервис для движения в заданную конфигурацию суставов.
        """
        if len(req.goal_positions) != 7 or not (0 < req.speed_factor <= 1):
            res.success = False
            res.message = 'Нужно 7 углов и 0 < скорость ≤ 1'
            return res
        try:
            q_path = self.planner.move('joints', req.goal_positions, velocity=req.speed_factor)  # type: ignore
            ok = self._send_to_controller(q_path)
            res.success = ok
            res.message = 'goal sent' if ok else 'controller busy'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_pose(self, req: ExecutePoseGoal.Request, res: ExecutePoseGoal.Response):
        """
        Сервис для движения в позицию/ориентацию ЭЭ в режиме ptp или lin.
        """
        if len(req.target_pose) != 6 or req.mode not in ('ptp', 'lin') or not (0 < req.speed_factor <= 1):
            res.success = False
            res.message = 'Неверный формат запроса'
            return res
        try:
            q_path = self.planner.move(req.mode, req.target_pose, velocity=req.speed_factor)  # type: ignore
            ok = self._send_to_controller(q_path)
            res.success = ok
            res.message = f'{req.mode} sent' if ok else 'controller busy'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _cb_goal_resp(self, fut):
        """
        Обработка ответа от action-сервера: принято или отклонено.
        """
        self.goal_handle = fut.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Цель отклонена контроллером')
            self._q_start = None
            self._q_goal = None
        else:
            self.goal_handle.get_result_async().add_done_callback(self._cb_result)

    def _cb_result(self, _):
        """
        Callback при завершении движения (успешно или отменено).
        """
        self.get_logger().info("Движение завершено")
        self._q_start = None
        self._q_goal = None

    def _cb_fb(self, fb):
        """
        Отслеживание и публикация прогресса движения (в процентах).
        """
        q_act = np.array(fb.feedback.actual.positions)
        if self._q_start is None or self._q_goal is None:
            return
        traveled = np.linalg.norm(q_act - self._q_start)
        total = np.linalg.norm(self._q_goal - self._q_start)
        prog = traveled / max(total, 1e-6)
        self.get_logger().info(f'Прогресс: {prog * 100:.1f}%')

    def _srv_cancel(self, _req, res):
        """
        Плавная отмена движения с использованием S-кривой торможения.
        """
        if self.goal_handle and self.q_goal_sent is not None:
            if self.goal_handle.status not in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING
            ):
                res.success = False
                res.message = "Цель уже завершена или отклонена"
                return res

            q_start = self.q_actual.copy()
            q_end = self.q_goal_sent.copy()
            steps = int(2.0 / self.DT)

            s_vals = self.planner.s_curve(steps)
            q_traj = [q_start * (1 - s) + q_end * s for s in s_vals]

            self.goal_handle.cancel_goal_async()
            self._send_to_controller(np.array(q_traj))

            self._q_start = q_start
            self._q_goal = q_end
            res.success = True
            res.message = 'Плавная остановка запущена'
        else:
            res.success = False
            res.message = 'Нет активной цели для остановки'
        return res

    def _srv_home(self, _req, res):
        """
        Сервис возврата в домашнюю конфигурацию (qz).
        """
        try:
            q_home = self.planner.robot.qz  # type: ignore
            q_path = self.planner.move('joints', q_home.tolist(), velocity=0.15)
            ok = self._send_to_controller(q_path)
            res.success = ok
            res.message = 'Домашняя поза отправлена' if ok else 'Контроллер занят'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_work(self, _req, res):
        """
        Сервис перехода в рабочую конфигурацию (qr).
        """
        try:
            q_work = self.planner.robot.qr  # type: ignore
            q_path = self.planner.move('joints', q_work.tolist(), velocity=0.15)
            ok = self._send_to_controller(q_path)
            res.success = ok
            res.message = 'Рабочая поза отправлена' if ok else 'Контроллер занят'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res


def main():
    rclpy.init()
    rclpy.spin(JointTrajectoryServer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
