import casadi as ca
from dataclasses import dataclass
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib

from models.baseModel import BaseModel


"""
x = (px, pz, vx, vz, pitch, vpitch)
a = (fl, fr)
"""

@dataclass
class DroneXZConfig:
    nx: int = 6
    nu: int = 2
    mass: float = 0.5
    inertia: float = 0.04
    d: float = 0.2
    gravity: float = 9.81


class DroneXZModel(BaseModel):
    def __init__(self, sampling_time):
        super().__init__(sampling_time)
        self.model_name = "DroneXZModel"
        self.model_config = DroneXZConfig()

        x = ca.MX.sym('x', self.model_config.nx)
        u = ca.MX.sym('u', self.model_config.nu)
        x_dot = ca.vertcat(
            x[2],  # \dot{px}
            x[3],  # \dot{pz}
            -(u[0] + u[1]) * ca.sin(x[4]) / self.model_config.mass,                               # \dot{vx}
            - self.model_config.gravity + (u[0] + u[1]) * ca.cos(x[4]) / self.model_config.mass,  # \dot{vz}
            x[5],  # \dot{pitch}
            (u[1] - u[0]) / self.model_config.inertia                                             # \dot{vpitch}
        )
        # set up integrator for discrete dynamics
        # multiply xdot by sampling time (time scaling), since casadi integrator integrates over [0,1] by default
        dae = {'x': x, 'p': u, 'ode': self._sampling_time * x_dot}

        self.I = ca.integrator('I', 'rk', dae)

        self.A_func = ca.Function('A_func', [x, u], [ca.jacobian(x_dot, x)])
        self.B_func = ca.Function('B_func', [x, u], [ca.jacobian(x_dot, u)])

        self.A_disc_func = ca.Function('A_disc_func', [x, u], [ca.jacobian(self.I(x0=x, p=u)['xf'], x)])
        self.B_disc_func = ca.Function('B_disc_func', [x, u], [ca.jacobian(self.I(x0=x, p=u)['xf'], u)])

        # create continuous and discrete dynamics
        self.f_cont = ca.Function('f_cont', [x, u], [x_dot])
        self.f_disc = ca.Function('f_disc', [x, u], [self.I(x0=x, p=u)['xf']])

    def linearizeContinuousDynamics(self, x, u):
        A = self.A_func(x, u).full()
        B = self.B_func(x, u).full()
        return A, B


    def linearizeDiscreteDynamics(self, x, u):
        A = self.A_disc_func(x, u).full()
        B = self.B_disc_func(x, u).full()
        return A, B


    def animateSimulation(self, x_trajectory, u_trajectory, additional_lines_or_scatters=None):
        fontsize = 16
        params = {
            'text.latex.preamble': r"\usepackage{gensymb} \usepackage{amsmath} \usepackage{amsfonts} \usepackage{cmbright}",
            'axes.labelsize': fontsize,
            'axes.titlesize': fontsize,
            'legend.fontsize': fontsize,
            'xtick.labelsize': fontsize,
            'ytick.labelsize': fontsize,
            "mathtext.fontset": "stixsans",
            "axes.unicode_minus": False,
        }
        matplotlib.rcParams.update(params)

        sim_length = u_trajectory.shape[1]
        fig, ax = plt.subplots()
        for i in range(sim_length+1):
            ax.set_aspect('equal')
            ax.set_xlim(-0.5, 2.0)
            ax.set_ylim(-0.5, 2.0)
            ax.set_xlabel(r'$p_{\mathrm{x}}$ (m)', fontsize=14)
            ax.set_ylabel(r'$p_{\mathrm{z}}$ (m)', fontsize=14)
            left_x = x_trajectory[0, i] - self.model_config.d * ca.cos(x_trajectory[4, i])
            left_z = x_trajectory[1, i] - self.model_config.d * ca.sin(x_trajectory[4, i])
            right_x = x_trajectory[0, i] + self.model_config.d * ca.cos(x_trajectory[4, i])
            right_z = x_trajectory[1, i] + self.model_config.d * ca.sin(x_trajectory[4, i])
            ax.plot(x_trajectory[0, :i+1], x_trajectory[1, :i+1], color="tab:gray", linewidth=2, zorder=0)
            ax.plot([left_x, right_x], [left_z, right_z], color="tab:blue", linewidth=5, zorder=1)
            ax.scatter(x_trajectory[0, i], x_trajectory[1, i], color="tab:gray", s=100, zorder=2)
            if i < sim_length:
                patch_fl = patches.Arrow(left_x, left_z, -0.1*u_trajectory[0, i]*ca.sin(x_trajectory[4, i]), 0.1*u_trajectory[0, i]*ca.cos(x_trajectory[4, i]), color="tab:green", width=0.2)
                patch_fr = patches.Arrow(right_x, right_z, -0.1*u_trajectory[1, i]*ca.sin(x_trajectory[4, i]), 0.1*u_trajectory[1, i]*ca.cos(x_trajectory[4, i]), color="tab:green", width=0.2)
                ax.add_patch(patch_fl)
                ax.add_patch(patch_fr)

            if additional_lines_or_scatters is not None:
                for key, value in additional_lines_or_scatters.items():
                    if value["type"] == "scatter":
                        ax.scatter(value["data"][0], value["data"][1], color=value["color"], s=value["s"], label=key, marker=value["marker"], zorder=3)
                    elif value["type"] == "line":
                        ax.plot(value["data"][0], value["data"][1], color=value["color"], linewidth=2, label=key)

            ax.set_title(f"Drone XZ Simulation: Step {i+1}")
            ax.legend()
            fig.subplots_adjust(bottom=0.15)
            if i < sim_length:
                plt.show(block=False)
                plt.pause(0.2)
            else:
                plt.show(block=True)
            ax.clear()
        return