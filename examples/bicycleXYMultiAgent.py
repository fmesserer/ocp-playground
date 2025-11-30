import casadi as ca
from dataclasses import dataclass, field
import matplotlib.pyplot as plt
import math
import numpy as np
import os
import sys

local_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(local_path, ".."))

import models.bicycleXYModel as bicycleXYModel


@dataclass
class GoalReachingCtrlConfig:
    max_delta: float = math.radians(15.0)
    max_V: float = 3.0
    Q: np.ndarray = field(default_factory=lambda: np.diag([1.0, 1.0, 1.0]))
    Q_e: np.ndarray = field(default_factory=lambda: np.diag([10.0, 10.0, 10.0]))
    R: np.ndarray = field(default_factory=lambda: np.diag([0.1, 0.1]))
    n_hrzn = 50
    num_agents = 2


class BicycleXYMultiAgentCtrl:
    def __init__(self, sampling_time: float, num_agents:int, model: bicycleXYModel.BicycleXYModel):
        self._sampling_time = sampling_time
        self._num_agents = num_agents
        self._model = model
        self._ctrl_config = GoalReachingCtrlConfig()
        self.x_sol = None
        self.u_sol = None
        self._goal_val = None

    def _define_ocp_variables(self):
        self._x_opt = ca.MX.sym('x', self._model.model_config.nx * self._num_agents, self._ctrl_config.n_hrzn+1, )
        self._u_opt = ca.MX.sym('u', self._model.model_config.nu * self._num_agents, self._ctrl_config.n_hrzn, )
        self._x_0_param = ca.MX.sym('x_0', self._model.model_config.nx * self._num_agents)
        self._goal_param = ca.MX.sym('goal', 3 * self._num_agents)

    def _setup_constraints(self):
        self._g = []
        self._lbg = []
        self._ubg = []
        # Input constraints
        nx = self._model.model_config.nx
        nu = self._model.model_config.nu
        for i in range(self._ctrl_config.n_hrzn):
            self._g.append(self._u_opt[0::nu, i])
            self._lbg.append(-self._ctrl_config.max_delta * np.ones(self._num_agents))
            self._ubg.append(self._ctrl_config.max_delta * np.ones(self._num_agents))
            self._g.append(self._u_opt[1::nu, i])
            self._lbg.append(np.zeros(self._num_agents,))
            self._ubg.append(self._ctrl_config.max_V * np.ones(self._num_agents))
        # System dynamics
        self._g.append(self._x_opt[:self._num_agents*nx, 0] - self._x_0_param)
        self._lbg.append(np.zeros(self._num_agents * nx,))
        self._ubg.append(np.zeros(self._num_agents * nx,))
        for i_agent in range(self._num_agents):
            for i in range(self._ctrl_config.n_hrzn):
                self._g.append(self._x_opt[i_agent*nx:(i_agent+1)*nx, i+1] - self._model.f_disc(self._x_opt[i_agent*nx:(i_agent+1)*nx, i], self._u_opt[i_agent*nu:(i_agent+1)*nu, i]))
        self._lbg.append(np.zeros(nx*self._ctrl_config.n_hrzn*self._num_agents,))
        self._ubg.append(np.zeros(nx*self._ctrl_config.n_hrzn*self._num_agents,))
        # Collision Avoidance Constraints
        for i_agent in range(self._num_agents):
            for j_agent in range(i_agent+1, self._num_agents):
                for i in range(1, self._ctrl_config.n_hrzn+1):
                    self._g.append(ca.sumsqr(self._x_opt[i_agent*nx:i_agent*nx+2, i] - self._x_opt[j_agent*nx:j_agent*nx+2, i]))
                    self._lbg.append((self._model.model_config.safety_radius * 2)**2)
                    self._ubg.append(ca.inf)
        return

    def _setup_obj_func(self):
        nx = self._model.model_config.nx
        nu = self._model.model_config.nu
        self._J = 0.0
        for i_agent in range(self._num_agents):
            for i in range(self._ctrl_config.n_hrzn):
                x_diff = self._x_opt[i_agent*nx:(i_agent+1)*nx, i] - self._goal_param[i_agent*nx:(i_agent+1)*nx, :]
                self._J += x_diff.T @ self._ctrl_config.Q @ x_diff # cost compensating for the distance to the goal
                self._J += self._u_opt[i_agent*nu:(i_agent+1)*nu, i].T @ self._ctrl_config.R @ self._u_opt[i_agent*nu:(i_agent+1)*nu, i]  # Control cost
            # Terminal cost
            x_diff = self._x_opt[i_agent*nx:(i_agent+1)*nx, -1] - self._goal_param[i_agent*nx:(i_agent+1)*nx, :]
            self._J += x_diff.T @ self._ctrl_config.Q_e @ x_diff
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
        nx = self._model.model_config.nx
        nu = self._model.model_config.nu
        x_0 = x_0.flatten()[:, np.newaxis]
        assert x_0.shape == (nx * self._num_agents, 1), f"Expected shape {(nx * self._num_agents, 1)}, but got {x_0.shape}"
        if self.x_sol is None:
            self.x_sol = np.tile(x_0, (1, self._ctrl_config.n_hrzn + 1))
        if self.u_sol is None:
            self.u_sol = np.zeros((nu * self._num_agents, self._ctrl_config.n_hrzn))
        if self._goal_val is None:
            raise ValueError("Goal value must be set before solving the OCP.")

        solution = self._solver(
            x0=ca.veccat(self.x_sol, self.u_sol),
            p=ca.vertcat(x_0, self._goal_val),
            lbg=ca.vertcat(*self._lbg),
            ubg=ca.vertcat(*self._ubg)
        )
        x_sol = solution['x'].full().flatten()[:(self._ctrl_config.n_hrzn+1) * nx * self._num_agents].reshape((nx * self._num_agents, self._ctrl_config.n_hrzn+1), order='F')
        u_sol = solution['x'].full().flatten()[(self._ctrl_config.n_hrzn+1) * nx * self._num_agents:].reshape((nu * self._num_agents, self._ctrl_config.n_hrzn, ), order='F')
        self.x_sol = x_sol
        self.u_sol = u_sol

        return u_sol[:, 0]

    @property
    def goal(self):
        return self._goal_val

    @goal.setter
    def goal(self, value):
        self._goal_val = value.flatten()[:, np.newaxis]


def closed_loop():
    sampling_time = 0.05
    sim_length = 10
    num_agents = 2
    x_init = np.array([3.2, 0.0, np.pi, 0.0, 0.0, 0.0, ])
    goal = np.array([0.0, 3.0, np.pi*0.5, 3.0, 3.0, np.pi*0.5, ])
    model = bicycleXYModel.BicycleXYModel(sampling_time)
    controller = BicycleXYMultiAgentCtrl(sampling_time, num_agents=num_agents, model=model)
    controller.setup_OCP()
    controller.goal = goal
    x_trajectory, u_trajectory = model.simulateClosedLoop(sim_length, x_init, controller.solve_OCP, num_agents=num_agents)
    additional_lines_or_scatters = {"Goal": {"type": "scatter", "data": [[goal[0], goal[3]], [goal[1], goal[4]]], "color": "tab:orange", "s": 100, "marker":"x"}}
    model.animateSimulation(x_trajectory, u_trajectory, num_agents=num_agents, additional_lines_or_scatters=additional_lines_or_scatters)


def open_loop():
    sampling_time = 0.05
    num_agents = 2
    x_init = np.array([0.0, 1.5, 0., 1.5, 0.0, np.pi*0.5, ])
    goal = np.array([3.2, 1.5, 0., 1.5, 3.0, np.pi*0.5, ])
    model = bicycleXYModel.BicycleXYModel(sampling_time)
    controller = BicycleXYMultiAgentCtrl(sampling_time, num_agents=num_agents, model=model)
    controller.setup_OCP()
    controller.goal = goal
    controller.solve_OCP(x_init)
    u_trajectory = controller.u_sol
    x_trajectory = controller.x_sol

    additional_lines_or_scatters = {"Goal": {"type": "scatter", "data": [[goal[0], goal[3]], [goal[1], goal[4]]], "color": "tab:orange", "s": 100, "marker":"x"}}
    model.animateSimulation(x_trajectory, u_trajectory, num_agents=num_agents, additional_lines_or_scatters=additional_lines_or_scatters)

if __name__ == "__main__":
    open_loop()