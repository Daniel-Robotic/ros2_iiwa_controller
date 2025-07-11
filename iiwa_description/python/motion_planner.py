import yaml
import numpy as np
from dataclasses import dataclass
from pathlib import Path
from spatialmath import SE3
from roboticstoolbox import ERobot, DHRobot, RevoluteDH
from roboticstoolbox.tools.trajectory import jtraj, trapezoidal
from roboticstoolbox.robot.Robot import Robot
from ament_index_python.packages import get_package_share_directory
from typing import Literal
from collections import OrderedDict
from spatialmath.base import transl, trotz


@dataclass
class RevoluteDHParamets:
    """
    Структура данных для хранения параметров одной суставной оси
    по модифицированной DH-модели (Denavit–Hartenberg).
    """
    alpha: float                  # Угол поворота вокруг оси X (в рад)
    d: float                      # Смещение вдоль оси Z
    a: float                      # Смещение вдоль оси X
    mass: float                   # Масса звена
    inertia: list[float]          # Инерционный тензор (вектор из 6 значений)
    lower: int                    # Нижний предел угла (в градусах)
    upper: int                    # Верхний предел угла (в градусах)
    velocity: int                 # Максимальная скорость сустава (град/с)
    effort: int                   # Максимальный момент (не используется здесь)
    initial_positions: float      # Начальная конфигурация (град)
    working_positions: float      # Рабочая конфигурация (град)


class KukaIiwaRobot(DHRobot):
    """
    Модель 7-связного манипулятора KUKA iiwa, построенная по DH-параметрам.
    Используется для планирования и вычисления траекторий.
    """

    def __init__(self):
        # 1. Загружаем параметры суставов из YAML-конфигурации
        revolute_params = self._load_config_params()

        # 2. Формируем список звеньев по DH-параметрам
        links = [
            RevoluteDH(
                d=params.d,
                a=params.a,
                alpha=params.alpha,
                qlim=[
                    np.deg2rad(params.lower),
                    np.deg2rad(params.upper)
                ],
                m=params.mass,
                I=params.inertia,
                G=1  # передаточное отношение (здесь 1)
            )
            for params in revolute_params
        ]
        
        # 3. Вызываем конструктор базового класса DHRobot
        super().__init__(
            links=links,
            name="KukaIiwaRobot",
            manufacturer="KUKA",
        )

        # 4. Устанавливаем инструмент на фланце (230 мм вдоль Z)
        mm = 1e-3
        tool_offset = 230 * mm
        flange = 107 * mm  # не используется явно

        self.tool = transl(0, 0, tool_offset) @ trotz(0)

        # 5. Конфигурации: рабочая, нулевая, скорости суставов
        self.qr = np.array([params.working_positions for params in revolute_params])
        self.qz = np.array([params.initial_positions for params in revolute_params]) 
        self.qd = np.array([np.deg2rad(params.velocity) for params in revolute_params])
    
        # 6. Добавляем конфигурации в словарь self.configurations
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qd", self.qd)
        
    def _load_config_params(self):
        """
        Загружает параметры суставов из YAML-файла.
        """
        pkg_path = get_package_share_directory("iiwa_description")
        file_path = "config/joint_setting.yaml"
        
        with open(Path(pkg_path, file_path), "r", encoding="utf-8") as config_f:
            config = yaml.load(config_f, Loader=yaml.SafeLoader)

            # Упорядочим суставы по имени (если необходимо)
            config = OrderedDict(sorted(config.items())) 
            
        # Преобразуем словари в dataclass-объекты
        revolute_params = [RevoluteDHParamets(**v) for _, v in config.items()]
        return revolute_params
        
        
class IiwaMotionPlanner:
    """
    Класс-планировщик для манипулятора KUKA iiwa.
    Поддерживает движение в суставное пространство, PTP и линейное движение в декартовом пространстве.
    """

    def __init__(self, 
                 robot: ERobot,
                 dt: float = 0.01):
        self.dt = dt                          # Шаг времени
        self.robot = robot                    # Модель робота
        self.q_current = self.robot.qz        # Начальная конфигурация (нулевая)  # type: ignore

    def euler_to_se3(self, x: float, y: float, z: float, 
                     a: float, b: float, c: float) -> SE3:
        """
        Преобразует декартову позицию и ориентацию (в RPY) в объект SE3.
        """
        rpy: list[float] = [a, b, c]
        return SE3(x, y, z) * SE3.RPY(rpy, order='xyz', unit='rad')  # type: ignore
        # return SE3(x, y, z) * SE3.RPY(rpy, order='zyx', unit='rad')  # type: ignore
    

    def updateCurrentPos(self, current_pos: list[float]):
        """
        Обновляет текущее положение робота в суставном пространстве.
        """
        self.q_current = current_pos

    def s_curve(self, steps: int) -> np.ndarray:
        """
        Генератор S-кривой (SmoothStep) от 0 до 1 для плавных траекторий.
        Используется для замедления или остановки.
        """
        t = np.linspace(0, 1, steps)
        return 35*t**4 - 84*t**5 + 70*t**6 - 20*t**7

    def move(self,
             mode: Literal["ptp", "lin", "joints"],
             target: list[float],
             velocity: float = 1.0) -> np.ndarray:
        """
        Генерация траектории движения.

        mode:
            - "ptp"    — движение в точку (с IK)
            - "lin"    — линейное движение ЭЭ (IK на каждом шаге)
            - "joints" — прямое задание углов

        target:
            - для "joints": список 7 углов
            - для "ptp" / "lin": список [x, y, z, roll, pitch, yaw]

        velocity:
            - множитель допустимой скорости (0 < v ≤ 1)
        """
        if velocity > 1 or velocity <= 0:
            raise ValueError(f"Error")

        q_target = []
        
        if mode in ["ptp", "lin"]:
            # Решаем ОЗК по целевой позе
            T_goal = self.euler_to_se3(*target)
            sol = self.robot.ikine_LM(T_goal, q0=self.q_current, joint_limits=True)  # type: ignore
            if not sol.success:
                raise ValueError("IK не сошелся для режима PTP.")
            q_target = sol.q
        elif mode == "joints":
            q_target = target
        else:
            raise ValueError(f"Неизвестный режим движения: {mode}")

        if self.q_current is None:
            raise RuntimeError("q_current не установлен. Вызовите updateCurrentPos().")

        # Расчёт времени движения и числа шагов
        delta = np.abs(q_target - self.q_current)  # type: ignore
        move_t = np.max(delta / (self.robot.qd * velocity))
        steps = max(int(move_t / self.dt), 2)

        # 1. Движение в суставное пространство или PTP
        if mode in ["ptp", "joints"]:
            traj = jtraj(self.q_current, q_target, steps)
            return traj.q

        # 2. Линейное движение — интерполяция SE3 + IK на каждом шаге
        T_start = self.robot.fkine(self.q_current)  # type: ignore
        T_target = self.euler_to_se3(*target)
        s_vals = trapezoidal(0, 1, steps).s

        q_traj = []
        q_prev = self.q_current.copy()

        for s in s_vals:
            T_i = T_start.interp(T_target, s)  # SE3-позы на пути

            if not isinstance(T_i, SE3):
                raise TypeError(f"Ожидался SE3, получено {type(T_i)}")

            sol = self.robot.ikine_LM(T_i, q0=q_prev, joint_limits=True)  # type: ignore
            if not sol.success:
                raise ValueError("IK не сошелся при LIN-движении.")
            
            q_prev = sol.q
            q_traj.append(q_prev)

        return np.array(q_traj)
