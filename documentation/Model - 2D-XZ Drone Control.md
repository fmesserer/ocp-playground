# (Model) 2D Drone

In this model, we consider a drone in the 2D xz-plane:

<img src="_misc/2DroneImage.png" width="350"/>

## Dynamics
The state and control vectors are

$$
x = \begin{bmatrix}
p \\
v \\
\phi \\
\dot{\phi}
\end{bmatrix} = \begin{bmatrix}
p_x \\
p_z \\
v_x \\
v_z \\
\phi \\
\dot{\phi}
\end{bmatrix}\in \mathbb{R}^6, \quad u = \begin{bmatrix}
u_\mathrm{l} \\
u_\mathrm{r}
\end{bmatrix} \in \mathbb{R}^2,
$$

with 2D position $p \in \mathbb{R}^2$, velocity $v \in \mathbb{R}^2$, orientation angle $\phi$ relative to the vertical axis and rotational velocity $\dot{\phi}$, such that $\dot{\phi} >0$ corresponds to counterclockwise rotation.
The drone is controlled using two positive and bounded rotor forces $0 \leq (u_\mathrm{l},u_\mathrm{r}) \leq u_\mathrm{max}$.

The following forces act on the drone:
- Rotor forces, with $u_\mathrm{l}, u_\mathrm{r}$ given as controls at distance $d = 5\,\mathrm{cm}$ from the center of gravity.

$$
F_p = \left(u_\mathrm{l} + u_\mathrm{r}\right) \begin{bmatrix}  \sin(\phi) \\
\cos(\phi)\end{bmatrix}
$$

- Gravity at the center of gravity

$$F_g = \begin{bmatrix}0 \\
-m g\end{bmatrix}$$
- Optional: aerodynamic drag force 

$$F_D(v, v_\mathrm{wind}) =\;?$$

The forces of the rotors not only move the drone, but also create a torque 
$$
M_p = \left(u_\mathrm{l} - u_\mathrm{r}\right) d,
$$

which rotates the drone around its center of mass. 

Overall, the resulting dynamics are:

$$
\begin{aligned}
\begin{bmatrix}
\dot{p} \\
\dot{v} \\
\dot{\phi} \\
\ddot{\phi}
\end{bmatrix} = \dot{x} = f(x,u) =  \begin{bmatrix}
v \\
m^{-1}(F_p + F_g) \\
\dot{\phi} \\
I^{-1} M_p
\end{bmatrix}.
\end{aligned}
$$


## Details

| State                                     | Symbol               | Unit          |
| ----------------------------------------- | -------------------- | ------------- |
| XZ - position of the drone                | $p \in \mathbb{R}^2$ | $\mathrm{m}$      |
| XZ - velocity of the drone                | $v \in \mathbb{R}^2$ | $\mathrm{m}\cdot \mathrm{s}^{-1}$     |
| Orientation relative to the vertical axis | $\phi $              | $\mathrm{rad}$           |
| Angular velocity                          | $\dot{\phi}$         | $\mathrm{rad}\cdot \mathrm{s}^{-1}$        |


| Control                                   | Symbol               | Unit          |
| ----------------------------------------- | -------------------- | ------------- |
| Force resulting from left rotor           | $u_\mathrm{l}\in \mathbb{R}$       | $\mathrm{N}$      |
| Force resulting from right rotor          | $u_\mathrm{r}\in \mathbb{R}$       | $\mathrm{N}$      |


| Parameter                   | Symbol | Value | Unit                      |
| --------------------------- | ------ | ----- | ------------------------- |
| distance to rotor           | $d$    | 0.20  | $\mathrm{m}$             |
| mass                        | $m$    | 0.50  | $\mathrm{kg}$             |
| rotational interia          | $I$    | 0.04  | $\mathrm{kg}\cdot\mathrm{m}^2$ |
| acceleration due to gravity | $g$    | 9.81  | $\mathrm{m}\cdot\mathrm{s}^{-2}$        |
