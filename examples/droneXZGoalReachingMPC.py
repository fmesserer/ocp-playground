import casadi as ca
from dataclasses import dataclass, field
import numpy as np
import os
import sys

local_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(local_path, ".."))

import models.droneXZModel as droneXZModel


@dataclass
class GoalReachingCtrlConfig:
    max_fl: float = 15.0
    max_fr: float = 15.0

    Q: np.ndarray = field(default_factory=lambda: np.diag([1.0, 1.0, 0.1, 0.1, 1.0, 0.1]))
    Q_e: np.ndarray = field(default_factory=lambda: np.diag([10.0, 10.0, 1.0, 1.0, 10.0, 1.0]))
    R: np.ndarray = field(default_factory=lambda: np.diag([0.01, 0.01]))
    n_hrzn = 20


class DroneXZGoalReachingCtrl:
    def __init__(self, sampling_time: float, model: droneXZModel.DroneXZModel):
        self._sampling_time = sampling_time
        self._model = model
        self._ctrl_config = GoalReachingCtrlConfig()
        self.x_sol = None
        self.u_sol = None
        self._goal_val = None


    def _define_ocp_variables(self):
        self._x_opt = ca.MX.sym('x', self._model.model_config.nx, self._ctrl_config.n_hrzn+1, )
        self._u_opt = ca.MX.sym('u', self._model.model_config.nu, self._ctrl_config.n_hrzn, )
        self._x_0_param = ca.MX.sym('x_0', self._model.model_config.nx)
        self._goal_param = ca.MX.sym('goal', 2)

    def _setup_constraints(self):
        self._g = []
        self._lbg = []
        self._ubg = []
        # Input constraints
        for i in range(self._ctrl_config.n_hrzn):
            self._g.append(self._u_opt[0, i])
            self._lbg.append(0.0)
            self._ubg.append(self._ctrl_config.max_fl)
            self._g.append(self._u_opt[1, i])
            self._lbg.append(0.0)
            self._ubg.append(self._ctrl_config.max_fr)
        # System dynamics
        self._g.append(self._x_opt[:, 0] - self._x_0_param)
        self._lbg.append(np.zeros(self._model.model_config.nx,))
        self._ubg.append(np.zeros(self._model.model_config.nx,))
        for i in range(self._ctrl_config.n_hrzn):
            self._g.append(self._x_opt[:, i+1] - self._model.f_disc(self._x_opt[:, i], self._u_opt[:, i]))
            self._lbg.append(np.zeros(self._model.model_config.nx,))
            self._ubg.append(np.zeros(self._model.model_config.nx,))
        return

    def _setup_obj_func(self):
        self._J = 0.0
        x_goal = ca.veccat(self._goal_param, ca.DM.zeros(4, 1))
        u_equilibrium = 0.5 * self._model.model_config.mass * self._model.model_config.gravity * ca.DM.ones(2, 1)
        for i in range(self._ctrl_config.n_hrzn):
            self._J += (self._x_opt[:, i] - x_goal).T @ self._ctrl_config.Q @ (self._x_opt[:, i] - x_goal)
            self._J += (self._u_opt[:, i] - u_equilibrium).T @ self._ctrl_config.R @ (self._u_opt[:, i] - u_equilibrium)
        # Terminal cost
        self._J += (self._x_opt[:, -1] - x_goal).T @ self._ctrl_config.Q_e @ (self._x_opt[:, -1] - x_goal)
        return

    def setup_OCP(self):
        self._define_ocp_variables()
        self._setup_constraints()
        self._setup_obj_func()
        ocp = {
            'x': ca.veccat(self._x_opt, self._u_opt),
            'p': ca.vertcat(self._x_0_param, self._goal_param),
            'g': ca.vertcat(*self._g),
            'f': self._J
        }
        opts = {'ipopt': {'print_level': 0, 'max_iter': 1000}, "print_time": False}
        self._solver = ca.nlpsol('solver', 'ipopt', ocp, opts)
        return

    def solve_OCP(self, x_0: np.ndarray):
        x_0 = x_0.flatten()[:, np.newaxis]
        if self.x_sol is None:
            self.x_sol = np.tile(x_0, (1, self._ctrl_config.n_hrzn + 1))
        if self.u_sol is None:
            self.u_sol = np.zeros((self._model.model_config.nu, self._ctrl_config.n_hrzn))
        if self._goal_val is None:
            raise ValueError("Goal value must be set before solving the OCP.")
        solution = self._solver(
            x0=ca.vertcat(self.x_sol.flatten(), self.u_sol.flatten()),
            p=ca.vertcat(x_0, self._goal_val),
            lbg=ca.vertcat(*self._lbg),
            ubg=ca.vertcat(*self._ubg)
        )
        x_sol = solution['x'].full().flatten()[:(self._ctrl_config.n_hrzn+1) * self._model.model_config.nx].reshape((self._model.model_config.nx, self._ctrl_config.n_hrzn+1), order='F')
        u_sol = solution['x'].full().flatten()[(self._ctrl_config.n_hrzn+1) * self._model.model_config.nx:].reshape((self._model.model_config.nu, self._ctrl_config.n_hrzn, ), order='F')
        self.x_sol = x_sol
        self.u_sol = u_sol

        return u_sol[:, 0]

    @property
    def goal(self):
        return self._goal_val

    @goal.setter
    def goal(self, value):
        self._goal_val = value.flatten()[:, np.newaxis]


def main():
    sampling_time = 0.05
    sim_length = 40
    x_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([1.0, 1.0])
    model = droneXZModel.DroneXZModel(sampling_time)
    controller = DroneXZGoalReachingCtrl(sampling_time, model)
    controller.setup_OCP()
    controller.goal = goal
    x_trajectory, u_trajectory = model.simulateClosedLoop(sim_length, x_init, controller.solve_OCP)
    additional_lines_or_scatters = {"Goal": {"type": "scatter", "data": [[goal[0]], [goal[1]]], "color": "tab:orange", "s": 100, "marker":"x"}}
    model.animateSimulation(x_trajectory, u_trajectory, additional_lines_or_scatters=additional_lines_or_scatters)
    return


if __name__ == "__main__":
    main()