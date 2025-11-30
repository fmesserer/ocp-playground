import casadi as ca
from dataclasses import dataclass
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np


from models.baseModel import BaseModel


"""
Consider the kinematic model only:
x = (px, py, yaw)
a = (delta, V)
"""


@dataclass
class BicycleXYConfig:
    nx: int = 3
    nu: int = 2
    lf: float = 0.5
    lr: float = 0.5
    safety_radius: float = 0.8

class BicycleXYModel(BaseModel):
    def __init__(self, sampling_time):
        super().__init__(sampling_time)
        self.model_name = "BicycleXYModel"
        self.model_config = BicycleXYConfig()

        x = ca.MX.sym('x', self.model_config.nx)
        u = ca.MX.sym('u', self.model_config.nu)

        beta = ca.arctan( self.model_config.lr * ca.tan(u[0]) / (self.model_config.lr + self.model_config.lf) )

        x_dot = ca.vertcat(
            u[1] * ca.cos(x[2] + beta),  # \dot{px}
            u[1] * ca.sin(x[2] + beta),  # \dot{py}
            u[1] * ca.sin(beta) / self.model_config.lr,  # \dot{yaw}
        )
        
        # set up integrator for discrete dynamics
        # multiply xdot by sampling time (time scaling), since casadi integrator integrates over [0,1] by default
        dae = {'x': x, 'p': u, 'ode': self._sampling_time * x_dot}
        self.I = ca.integrator('I', 'rk', dae)

        # create continuous and discrete dynamics
        self.f_cont = ca.Function('f_cont', [x, u], [x_dot])
        self.f_disc = ca.Function('f_disc', [x, u], [self.I(x0=x, p=u)['xf']])


    def animateSimulation(self, x_trajectory, u_trajectory, num_agents:int=1, additional_lines_or_scatters=None):
        wheel_long_axis = 0.4
        wheel_short_axis = 0.1

        sim_length = u_trajectory.shape[1]
        _, ax = plt.subplots()

        nx = self.model_config.nx
        nu = self.model_config.nu

        for i in range(sim_length+1):
            ax.set_aspect('equal')
            ax.set_xlim(-1, 5)
            ax.set_ylim(-1, 5)
            ax.set_xlabel(r'$p_{\mathrm{x}}$ (m)', fontsize=14)
            ax.set_ylabel(r'$p_{\mathrm{y}}$ (m)', fontsize=14)

            for i_agent in range(num_agents):
                front_x = x_trajectory[i_agent*nx, i] + self.model_config.lf * ca.cos(x_trajectory[i_agent*nx+2, i])
                front_y = x_trajectory[i_agent*nx+1, i] + self.model_config.lf * ca.sin(x_trajectory[i_agent*nx+2, i])
                rear_x = x_trajectory[i_agent*nx, i] - self.model_config.lr * ca.cos(x_trajectory[i_agent*nx+2, i])
                rear_y = x_trajectory[i_agent*nx+1, i] - self.model_config.lr * ca.sin(x_trajectory[i_agent*nx+2, i])
                ax.plot(x_trajectory[i_agent*nx, :i+1], x_trajectory[i_agent*nx+1, :i+1], color="tab:gray", linewidth=2, zorder=0)
                ax.scatter(x_trajectory[i_agent*nx, i], x_trajectory[i_agent*nx+1, i], color="tab:gray", s=50, zorder=2)
                ax.plot([front_x, rear_x], [front_y, rear_y], color="tab:blue", linewidth=3, zorder=1)
                if i < sim_length:
                    wheel_f = patches.Ellipse((front_x, front_y), wheel_long_axis, wheel_short_axis, angle=math.degrees(x_trajectory[i_agent*nx+2, i] + u_trajectory[0, i]), color="tab:green", label="Wheels" if i_agent == 0 else None)
                    wheel_r = patches.Ellipse((rear_x, rear_y), wheel_long_axis, wheel_short_axis, angle=math.degrees(x_trajectory[i_agent*nx+2, i]), color="tab:green")
                    ax.add_patch(wheel_f)
                    ax.add_patch(wheel_r)
                safety_circle = patches.Circle((x_trajectory[i_agent*nx, i], x_trajectory[i_agent*nx+1, i]), self.model_config.safety_radius, color="tab:orange", alpha=0.3)
                ax.add_patch(safety_circle)
            if additional_lines_or_scatters is not None:
                for key, value in additional_lines_or_scatters.items():
                    if value["type"] == "scatter":
                        ax.scatter(value["data"][0], value["data"][1], color=value["color"], s=value["s"], label=key, marker=value["marker"])
                    elif value["type"] == "line":
                        ax.plot(value["data"][0], value["data"][1], color=value["color"], linewidth=2, label=key)
            ax.set_title(f"Bicycle Simulation: Step {i+1}")
            ax.legend()
            if i < sim_length:
                plt.show(block=False)
                plt.pause(0.3)
            else:
                plt.show(block=True)
            ax.clear()
        return
    