import casadi as ca
from dataclasses import dataclass
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib
import matplotlib.animation as animation
import numpy as np 

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


class DroneXZModel:
    def __init__(self, sampling_time):
        self._sampling_time = sampling_time
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
            (u[1] - u[0]) * self.model_config.d / self.model_config.inertia                                             # \dot{vpitch}
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

    def animateSimulation(self, x_trajectory, u_trajectory, additional_lines_or_scatters=None, save_path: str = None):
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
        ax.set_aspect('equal')
        ax.set_xlim(-0.5, 2.0)
        ax.set_ylim(-0.5, 2.0)
        ax.set_xlabel(r'$p_{\mathrm{x}}$ in m', fontsize=14)
        ax.set_ylabel(r'$p_{\mathrm{z}}$ in m', fontsize=14)

        # static artists
        path_line, = ax.plot([], [], color="tab:gray", linewidth=2, zorder=0)
        body_line, = ax.plot([], [], color="tab:blue", linewidth=5, zorder=1)
        center_scatter = ax.scatter([], [], color="tab:gray", s=100, zorder=2)
        # thrust patches (will create Arrow patches each frame to match original style)
        thrust_fl = None
        thrust_fr = None

        added_artists = []
        if additional_lines_or_scatters is not None:
            for key, value in additional_lines_or_scatters.items():
                if value["type"] == "scatter":
                    sc = ax.scatter(value["data"][0], value["data"][1], color=value.get("color", "k"), s=value.get("s", 20), label=key, marker=value.get("marker", ","), zorder=3)
                    added_artists.append(sc)
                elif value["type"] == "line":
                    ln, = ax.plot(value["data"][0], value["data"][1], color=value.get("color", "k"), linewidth=2, label=key)
                    added_artists.append(ln)

        if added_artists:
            ax.legend()
        fig.subplots_adjust(bottom=0.15)

        interval = max(50, int(self._sampling_time * 1000 * 2.0))

        def init():
            path_line.set_data([], [])
            body_line.set_data([], [])
            center_scatter.set_offsets(np.empty((0, 2)))
            # no arrows at init
            return [path_line, body_line, center_scatter] + added_artists

        def update(i):
            left_x = float(x_trajectory[0, i] - self.model_config.d * np.cos(x_trajectory[4, i]))
            left_z = float(x_trajectory[1, i] - self.model_config.d * np.sin(x_trajectory[4, i]))
            right_x = float(x_trajectory[0, i] + self.model_config.d * np.cos(x_trajectory[4, i]))
            right_z = float(x_trajectory[1, i] + self.model_config.d * np.sin(x_trajectory[4, i]))
            path_line.set_data(x_trajectory[0, :i+1], x_trajectory[1, :i+1])
            body_line.set_data([left_x, right_x], [left_z, right_z])
            center_scatter.set_offsets(np.array([[float(x_trajectory[0, i]), float(x_trajectory[1, i])]]))

            nonlocal thrust_fl, thrust_fr
            # remove previous arrows if present
            if thrust_fl is not None:
                try:
                    thrust_fl.remove()
                except Exception:
                    pass
                thrust_fl = None
            if thrust_fr is not None:
                try:
                    thrust_fr.remove()
                except Exception:
                    pass
                thrust_fr = None

            if i < sim_length:
                v_fl = float(u_trajectory[0, i])
                v_fr = float(u_trajectory[1, i])
                # original arrow scaling (match previous implementation)
                dx_fl = -0.1 * v_fl * np.sin(float(x_trajectory[4, i]))
                dz_fl = 0.1 * v_fl * np.cos(float(x_trajectory[4, i]))
                dx_fr = -0.1 * v_fr * np.sin(float(x_trajectory[4, i]))
                dz_fr = 0.1 * v_fr * np.cos(float(x_trajectory[4, i]))
                thrust_fl = patches.Arrow(left_x, left_z, dx_fl, dz_fl, color="tab:green", width=0.2)
                thrust_fr = patches.Arrow(right_x, right_z, dx_fr, dz_fr, color="tab:green", width=0.2)
                ax.add_patch(thrust_fl)
                ax.add_patch(thrust_fr)

            if added_artists:
                for a in added_artists:
                    pass

            ax.set_title(f"Drone XZ Simulation: Step {i+1}")
            return [path_line, body_line, center_scatter, thrust_fl, thrust_fr] + added_artists

        anim = animation.FuncAnimation(fig, update, frames=range(sim_length + 1), init_func=init, interval=interval, blit=False, repeat=True)

        # save if requested
        if save_path is not None:
            try:
                fps = max(1, int(round(1000.0 / float(interval))))
                print(f"Saving drone animation to {save_path} (fps={fps})")
                writer = animation.PillowWriter(fps=fps)
                anim.save(save_path, writer=writer)
            except Exception as e:
                print(f"Failed to save drone animation to {save_path}: {e}")

        plt.show()
        return anim, fig

    def plotSimulation(self, x_trajectory: np.ndarray, u_trajectory: np.ndarray, figsize=(8, 10)):
        """Plot states and controls over time for the drone.
        """
        dt = self._sampling_time
        sim_length = u_trajectory.shape[1]

        t_x = np.arange(sim_length + 1) * dt

        fig, axs = plt.subplots(6, 1, figsize=figsize, constrained_layout=True)

        # Positions px, pz
        axs[0].plot(t_x, x_trajectory[0, :], label=r'$p_{\mathrm{x}}$')
        axs[0].plot(t_x, x_trajectory[1, :], label=r'$p_{\mathrm{z}}$')
        axs[0].set_ylabel(r'position $p$ in m')
        axs[0].legend()

        # Velocities vx, vz
        axs[1].plot(t_x, x_trajectory[2, :], label=r'$v_{\mathrm{x}}$')
        axs[1].plot(t_x, x_trajectory[3, :], label=r'$v_{\mathrm{z}}$')
        axs[1].set_ylabel(r'velocity $v$ in m/s')
        axs[1].legend()

        # Pitch
        axs[2].plot(t_x, x_trajectory[4, :], label='pitch')
        axs[2].set_ylabel(r'pitch $\theta$ in rad')

        # Pitch rate
        axs[3].plot(t_x, x_trajectory[5, :], label='pitch rate')
        axs[3].set_ylabel(r'pitch rate $\dot{\theta}$ in rad/s')

        # Controls: fl
        u_fl_plot = np.concatenate((np.asarray(u_trajectory[0, :]).flatten(), [np.nan]))
        axs[4].step(t_x, u_fl_plot, where='post', label='fl')
        axs[4].set_ylabel(r'left thrust $f_{\mathrm{l}}$ in N')

        # Controls: fr
        u_fr_plot = np.concatenate((np.asarray(u_trajectory[1, :]).flatten(), [np.nan]))
        axs[5].step(t_x, u_fr_plot, where='post', label='fr')
        axs[5].set_ylabel(r'right thrust $f_{\mathrm{r}}$ in N')
        axs[5].set_xlabel('time in s')

        fig.suptitle('Drone states and controls')

        # Set x-axis limits exactly
        x_min = t_x[0]
        x_max = t_x[-1]
        for ax in axs:
            ax.set_xlim(x_min, x_max)

        return fig, axs