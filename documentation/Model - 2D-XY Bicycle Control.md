# (Model) Kinematic bicycle model

<img src="_misc/BicycleXYfigure.png" width="500"/>

## Dynamics
We consider a kinematic bicycle model.
The state and control vectors are
$$
x = \begin{bmatrix}
p_x \\
p_y \\
\theta
\end{bmatrix} \in \mathrm{R}^3, \quad u = \begin{bmatrix}
\delta \\
V
\end{bmatrix} \in \mathrm{R}^2
$$
with position $p_x$, $p_y$, and heading angle $\theta$ relative to the $x$-axis. The vehicle is controlled by choosing the steering angle $\delta$ and the velocity $V$.
The resulting dynamics are
$$
\begin{bmatrix}
\dot{p}_ x \\
\dot{p}_ y \\
\dot{\theta}
\end{bmatrix} = \dot{x} = f(x,u) =  \begin{bmatrix}
V \cos(\theta + \beta) \\
V \sin(\theta + \beta) \\
\frac{V}{l_\mathrm{r}}\sin(\beta(\delta
))
\end{bmatrix}
$$

The side-slip angle $\beta$ is 
$$
\beta(\delta) = \arctan\left(\frac{l_\mathrm{r} \tan(\delta)}{l_\mathrm{r} + l_\mathrm{f}}\right)
$$
where $l_\mathrm{r},l_\mathrm{f}$ are the distances of the center of mass from the front resp. rear wheels. In the plot we have $L = l_\mathrm{r} + l_\mathrm{f}$.


## Details

| State                                     | Symbol               | Unit  |
| ----------------------------------------- | -------------------- | ----- |
| Position of center of mass                | $p \in \mathbb{R}^2$ | $\mathrm{m}$      |
| Orientation relative to the vertical axis | $\phi \in \mathbb{R}$               | $\mathrm{rad}$    |

| Control                                     | Symbol               | Unit  |
| ----------------------------------------- | -------------------- | ----- |
| Speed                                         | $V \in \mathbb{R}$ | $\mathrm{m} \mathrm{s}^{-1}$      |
| Steering angle | $\delta \in \mathbb{R}$               | $\mathrm{rad}$    |


| Parameter                              | Symbol         | Value | Unit          |
| -------------------------------------- | -------------- | ----- | ------------- |
| distance center of mass to front wheel | $l_\mathrm{f}$ | 0.5   | $\mathrm{m}$              |
| distance center of mass to front wheel | $l_\mathrm{r}$ | 0.5   | $\mathrm{m}$ |
