# (Method) Open Loop Planning

<img src="_misc/openLoop.svg" width="800"/>



We consider a system modeled by an ordinary differential equation 

$$
\begin{aligned}
	\dot{x} = f(x,u),
\end{aligned}
$$

with state $x\in \mathbb{R}^{n_x}$, and control $u\in \mathbb{R}^{n_u}$.
We want to find a find a control strategy $u(t)$ such that the planned trajectory $x(t)$ is optimal with respect to some cost function.
For this we use *direct methods*, which involve first discretizing the dynamics and their solution and then formulating the optimal control task into a nonlinear program.
## Discrete Dynamics

We approximate the continuous state trajectory $x(t)$ on grid points $t_0, \dots, t_k, t_{k+1}, \dots, t_N$ as $x(t_k) \approx x_k$.
Also, parametrize the control trajectory as piecewise constant over each interval: $u(t) = u_k, \forall  t \in [t_k, t_{k+1}]$, with intervals of constant duration, $h = t_{k+1} - t_k$.
This defines the discretized dynamics

$$
\begin{aligned}
	x_{k+1} = F(x_k, u_k),
\end{aligned}
$$

which can be obtained from the continuous-time ODE via numerical integration.
A commonly used integration method is the Runge-Kutta method of order 4:

$$
\begin{aligned}
	k_1 &= f(x_k, u_k) \\
	k_2 &= f\left(x_k + \frac{h}{2}k_1, u_k\right) \\
	k_3 &= f\left(x_k + \frac{h}{2}k_2, u_k\right) \\
	k_4 &= f\left(x_k + h k_3, u_k\right) \\
	x_{k+1} &= x_k + \frac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4) \\
		    &= F(x_k, u_k).
\end{aligned}
$$



## Discrete Optimal Control Problem
<img src="_misc/DOCP.png" width="700"/>

Given the system model and constraints, a quite generic discrete time optimal control problem can be formulated as the following constrained NLP:

$$
\begin{aligned}
\min_{x_0,u_0, \dots, u_{N-1}, x_N} &\sum_{k=0}^{N-1} l_k(x_k,u_k) + E(x_N) \\
\text{s.t.}\quad & 0 = x_0 - \bar{x}_0, \\
&  0 = x_{k+1} - F(x_k, u_k), \quad &k=0,\dots,N-1, \\
&  0 \geq h(x_k, u_k), \quad &k=0,\dots,N-1.
\end{aligned}
$$

The decision variables of the problem contain the *discrete* state and control trajectories on the time grid. We have $N+1$ variables 

$$
x_0,x_1, \dots, x_N \in \mathbb{R}^{n_x}
$$

for the state trajectory, and $N$ variables for the control trajectory,

$$
u_0, u_1, \dots, u_{N-1} \in \mathbb{R}^{n_u}.
$$

The planned trajectory should satisfy the discrete dynamics of the system, and should start at some initial state $\bar{x}_0 \in \mathbb{R}^{n_x}$, yielding the equality constraints

$$
\begin{aligned}& 0 = x_0 - \bar{x}_ 0, \\
&  0 = x_{k+1} - F(x_k, u_k), \quad &k=0,\dots,N-1. \end{aligned}
$$

The cost function is divided into a *stage cost*  $l(x_k, u_k)$ for each interval and a terminal cost $E(x_N)$ for the terminal node. A very common example is a *tracking cost* 

$$
\begin{aligned}\sum_{k=0}^{N-1} (x_k - \bar{x}_ k)^\top Q (x_k - \bar{x}_ k) + (u_k - \bar{u}_ k)^\top R (u_k - \bar{u}_ k), \end{aligned}
$$

with given reference trajectories $\bar{x}_0$, $\dots$,  $\bar{x}_N$ and  $\bar{u}_0$, $\dots$, $\bar{u}_{N-1}$.
The different entries of the state and control vector can be traded-off against each other via the  (typically diagonal) weighting matrices $Q$ and $R$.

Furthermore, we can demand that trajectory should satisfy some constraints, for example simple bounds. This is expressed via stagewise inequality constraints, 

$$
h(x_k, u_k) \leq 0.
$$

## Practical Solution of the Nonlinear Program
The nonlinear program above is of the general form

$$
\begin{aligned}\min_w\quad& f(w) \\
\text{s.t.}\quad& 0 = g(w) \\
& 0 \leq h(w)
\end{aligned}
$$

with variables $w$, objective function $f$, equality constraints $g$ and inequality constraints $h$. Such an NLP can for example be formulated using `CasADi`  and solved with `IPOPT`.


## Further Literature:
- **Moritz Diehl and Sébastien Gros**, _Numerical Optimal Control_. Available online: [http://www.syscop.de/numericaloptimalcontrol](http://www.syscop.de/numericaloptimalcontrol).

- **J. B. Rawlings, D. Q. Mayne, and M. M. Diehl**, *Model Predictive Control: Theory, Computation, and Design*, 2nd edition, Nob Hill, 2017.
