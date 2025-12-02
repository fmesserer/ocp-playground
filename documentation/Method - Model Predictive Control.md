# (Method) Model Predictive Control

Model predictive control is an advanced control method, which finds a closed-loop control $u(x)$ by solving a discrete optimal control problem in every iteration, given an (estimate $\hat{x}$ of the) current system state $x$.

<img src="_misc/closedLoop.svg" width="600"/>

The problem solved is given by the following constrained nonlinear program:

$$
\begin{aligned}
\min_{x_0,u_0,\dots,u_{N-1}, x_N} &\sum_{k=0}^{N-1} l_k(x_k,u_k) + E(x_N) \\
\text{s.t.}\quad & 0 = x_0 - x, \\
&  0 = x_{k+1} - F(x_k, u_k), \quad &k=0,\dots,N-1, \\
&  0 \geq h(x_k, u_k), \quad &k=0,\dots,N-1.
\end{aligned}
$$

More details can be found in the [Open Loop Planning](../documentation/Method%20-%20Open%20Loop%20Planning.md) page.

After its solution, the first control $u_0$ is applied to the system. In the next time step, the optimization problem is solved again with the new state $x$ of the system as initial state $x_0$.


## Further Literature:
- **Moritz Diehl and Sébastien Gros**, _Numerical Optimal Control_. [https://www.syscop.de/files/2024ws/NOC/book-NOCSE.pdf](https://www.syscop.de/files/2024ws/NOC/book-NOCSE.pdf).

- **J. B. Rawlings, D. Q. Mayne, and M. M. Diehl**, *Model Predictive Control: Theory, Computation, and Design*, 2nd edition, Nob Hill, 2017. [https://sites.engineering.ucsb.edu/~jbraw/mpc/](https://sites.engineering.ucsb.edu/~jbraw/mpc/).

