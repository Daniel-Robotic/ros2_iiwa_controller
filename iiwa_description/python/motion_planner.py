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

@dataclass
class RevoluteDHParamets:
    alpha: float
    d: float
    a: float
    mass: float
    inertia: list[float]
    lower: int
    upper: int
    velocity: int
    effort: int
    initial_positions: float
    working_positions: float

        
class KukaIiwaRobot(DHRobot):
    def __init__(self):

        revolute_params = self._load_config_params()
        links = [RevoluteDH(d=params.d,
                            a=params.a,
                            alpha=params.alpha,
                            qlim=[np.deg2rad(params.lower), np.deg2rad(params.upper)],
                            m=params.mass,
                            I=params.inertia,
                            G=1) for params in revolute_params]
        
        super().__init__(
            links=links,
            name="KukaIiwaRobot",
            manufacturer="KUKA",
        )

        self.qr = np.array([params.working_positions for params in revolute_params])
        self.qz = np.array([params.initial_positions for params in revolute_params]) 
        self.qd = np.array([np.deg2rad(params.velocity) for params in revolute_params])
    
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qd", self.qd)
        
    def _load_config_params(self):
        pkg_path = get_package_share_directory("iiwa_description")
        file_path = "config/joint_setting.yaml"
        
        with open(Path(pkg_path, file_path), "r", encoding="utf-8") as config_f:
            config = yaml.load(config_f, Loader=yaml.SafeLoader)
            config = OrderedDict(sorted(config.items())) 
            
        revolute_params = [RevoluteDHParamets(**v) for _, v in config.items()]

        return revolute_params
        
        
class IiwaMotionPlanner:
    def __init__(self, 
                 robot: ERobot,
                 dt: float = 0.01):
        
        self.dt = dt
        self.robot = robot
        self.q_current = self.robot.qz
        
    def euler_to_se3(self, x, y, z, a, b, c) -> SE3:
        return SE3(x, y, z) * SE3.RPY([a, b, c], order='zyx')
    
    def updateCurrentPos(self, current_pos: list[float]):
        self.q_current = current_pos
    
    def s_curve(self, steps: int) -> np.ndarray:
        """
        S-кривая (smoothstep) от 0 до 1.
        """
        t = np.linspace(0, 1, steps)
        return 35*t**4 - 84*t**5 + 70*t**6 - 20*t**7

    
    def move(self,
             mode: Literal["ptp", "lin", "joints"],
             target: list[float],
             velocity: float = 1.0) -> np.ndarray:
        
        if velocity > 1 or velocity <= 0:
            raise ValueError(f"Error")
        
        q_target = []
        if mode in ["ptp", "lin"]:
            T_goal = self.euler_to_se3(*target)
            sol = self.robot.ikine_LM(T_goal, q0=self.q_current, joint_limits=True)
            if not sol.success:
                raise ValueError("IK failed for PTP mode.")
            q_target = sol.q
        elif mode == "joints":
            q_target = target
        else:
            raise ValueError(f"Unknown mode: {mode}")
        
        if self.q_current is None:
            raise RuntimeError("Current joint position is None. Make sure updateCurrentPos() was called.")
        
        delta = np.abs(q_target - self.q_current)
        move_t = np.max(delta / (self.robot.qd * velocity))
        steps = max(int(move_t / self.dt), 2)
        
        if mode in ["ptp", "joints"]:
            traj = jtraj(self.q_current, q_target, steps)
            return traj.q
        
        T_start = self.robot.fkine(self.q_current)
        T_target = self.euler_to_se3(*target)
        s_vals = trapezoidal(0, 1, steps).s
        
        q_traj = []
        q_prev = self.q_current.copy()
        
        for s in s_vals:
            T_i = T_start.interp(T_target, s)
            
            if not isinstance(T_i, SE3):
                raise TypeError(f"Expected SE3, got {type(T_i)}")
        
            sol = self.robot.ikine_LM(T_i, q0=q_prev, joint_limits=True)
            if not sol.success:
                raise ValueError("IK failed during LIN trajectory.")
                
            q_prev = sol.q
            q_traj.append(q_prev)
            
        return np.array(q_traj)

