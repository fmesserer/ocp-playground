import casadi as ca
from dataclasses import dataclass, field
import numpy as np
import math
import os
import sys

local_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(local_path, ".."))

import models.rocketXZModel as rocketXZModel


@dataclass
class RocketOCPConfig:
    n_hrzn: int = 50
    Q: np.ndarray = field(default_factory=lambda: np.diag([10.0, 10.0, 1.0, 1.0, 50.0, 1.0]))
    Q_e: np.ndarray = field(default_factory=lambda: np.diag([50.0, 50.0, 1.0, 1.0, 100.0, 1.0]))
    R: np.ndarray = field(default_factory=lambda: np.diag([0.01, 0.01]))
    max_T: float = 15
    min_T: float = 0
    max_delta: float = 0.5


class RocketXZSimpleOCP:
    def __init__(self, sampling_time: float, model: rocketXZModel.RocketXZModel):
        self._sampling_time = sampling_time
        self._model = model
        self._cfg = RocketOCPConfig()

        self.x_sol = None
        self.u_sol = None
        self._x_0_param = None
        self._goal_param = None

    def _define_ocp_variables(self):
        nx = self._model.model_config.nx
        nu = self._model.model_config.nu
        N = self._cfg.n_hrzn
        self._x_opt = ca.MX.sym('x', nx, N+1)
        self._u_opt = ca.MX.sym('u', nu, N)
        self._x_0_param = ca.MX.sym('x_0', nx)
        self._goal_param = ca.MX.sym('goal', nx)

    def _setup_constraints(self):
        nx = self._model.model_config.nx
        nu = self._model.model_config.nu
        N = self._cfg.n_hrzn
        self._g = []
        self._lbg = []
        self._ubg = []

        # input constraints (simple bound enforcement via inequality constraints)
        for k in range(N):
            self._g.append(self._u_opt[0, k])
            self._lbg.append(self._cfg.min_T)
            self._ubg.append(self._cfg.max_T)
            self._g.append(self._u_opt[1, k])
            self._lbg.append(-self._cfg.max_delta)
            self._ubg.append(self._cfg.max_delta)

        # initial condition equality
        self._g.append(self._x_opt[:, 0] - self._x_0_param)
        self._lbg.append(np.zeros(nx,))
        self._ubg.append(np.zeros(nx,))

        # dynamics equality constraints
        for k in range(N):
            x_next = self._model.f_disc(self._x_opt[:, k], self._u_opt[:, k])
            self._g.append(self._x_opt[:, k+1] - x_next)
        self._lbg.append(np.zeros(nx * N,))
        self._ubg.append(np.zeros(nx * N,))

    def _setup_obj_func(self):
        nx = self._model.model_config.nx
        nu = self._model.model_config.nu
        N = self._cfg.n_hrzn
        J = 0.0
        for k in range(N):
            x_err = self._x_opt[:, k] - self._goal_param
            J += ca.mtimes([x_err.T, self._cfg.Q, x_err])
            u = self._u_opt[:, k]
            J += ca.mtimes([u.T, self._cfg.R, u])
        # terminal cost
        x_err = self._x_opt[:, -1] - self._goal_param
        J += ca.mtimes([x_err.T, self._cfg.Q_e, x_err])
        self._J = J

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

    def solve_OCP(self, x_0: np.ndarray):
        nx = self._model.model_config.nx
        nu = self._model.model_config.nu
        N = self._cfg.n_hrzn
        x_0 = x_0.flatten()[:, np.newaxis]
        assert x_0.shape == (nx, 1)

        # warm start
        if self.x_sol is None:
            self.x_sol = np.tile(x_0, (1, N+1))
        if self.u_sol is None:
            self.u_sol = np.zeros((nu, N))

        solution = self._solver(
            x0=ca.veccat(self.x_sol, self.u_sol),
            p=ca.vertcat(x_0, self._goal_param),
            lbg=ca.vertcat(*self._lbg),
            ubg=ca.vertcat(*self._ubg)
        )

        sol_vec = solution['x'].full().flatten()
        x_len = (N+1) * nx
        x_sol = sol_vec[:x_len].reshape((nx, N+1), order='F')
        u_sol = sol_vec[x_len:].reshape((nu, N), order='F')
        self.x_sol = x_sol
        self.u_sol = u_sol
        return u_sol[:, 0]


def open_loop():
    sampling_time = 0.05
    model = rocketXZModel.RocketXZModel(sampling_time)
    ocp = RocketXZSimpleOCP(sampling_time, model)
    ocp.setup_OCP()

    # initial state: px, pz, vx, vz, pitch, vpitch
    x_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # goal: some forward and up position with nose up
    goal = np.array([3.0, 3.0, 0.0, 0.0, 0.0, 0.0])
    ocp._goal_param = goal[:, np.newaxis]

    # solve once for open-loop plan
    ocp.solve_OCP(x_init)
    u_trajectory = ocp.u_sol
    x_trajectory = ocp.x_sol

    # simulate open loop using model to ensure compatibility
    x_traj_sim = model.simulateOpenLoop(x_init, u_trajectory)

    additional_lines_or_scatters = {"Goal": {"type": "scatter", "data": [[goal[0]], [goal[1]]], "color": "tab:orange", "s": 100, "marker":"x"}}
    model.animateSimulation(x_traj_sim, u_trajectory, additional_lines_or_scatters=additional_lines_or_scatters)


if __name__ == "__main__":
    open_loop()
