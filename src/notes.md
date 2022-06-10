## Quadrotor dynamic

The thrust `F (Newton)` is proportional to the `angular velocity ω (rad/s)` via a constant k_f
```
F_i = k_f * ω_i²
```

The Drag Moment `M (Newton meters)` is proportional to the `angular velocity ω (rad/s)` via a constant k_m
```
M_i = k_m * ω_i²
```

The total resultant (net) Force
```
F = F₁ + F₂ + F₃ + F₄ - m * g * a₃
```
With `a₃` being the vector component in the Z direction in the world frame.  

The total resultant (net) Moment
```
M = r₁ * F₁ + r₂ * F₂ + r₃ * F₃ + r₄ * F₄ + M₁ + M₂ + M₃ + M₄
```  

At hover state, `F = 0` and `M = 0`



Reflexion
- F_i = k_f * ω_i² : 4 thrusts (f1...f4)
- M_i = k_m * ω_i² : 4 moments (M1...M4)
- r = position vector of the center of mass (vector that link origine on world - coordinate to origin of body coordinate)
- 3d world coordinate = a1, a2, a3
- 3d body coordinate = b1, b2, b3
- ZXY convention euler angle = rot about psi, then phi, then theta
- so R = rot(z, psi) * rot(x, phi) * rot(y, theta)
- we use newton-euler equations to predict the net acceleration

newton-euler equations

how to find center of mass of a system of N particles :
- weight a position vector Pi by the mass mi of each point : mi * Pi
- Sum all the weighted position : Sum(mi * Pi)    [from 1 to N]
- divide by the total sum m : 1/m * Sum(mi * Pi)  [from 1 to N]
- center of mass rc = 1/m * Sum(mi * Pi)  [from 1 to N]  -> rc is a new position vector

euler eq of motion:
```rust
I = [
    [I11, 0, 0],
    [0, I22, 0],
    [0, 0, I33]]

w_dot = [w1_dot, w2_dot, w3_dot]  // col vector

w = [w1, w2, w3] = [p, q, r]   // col vector
  = [
    [cos(theta), 0, -cos(phi)*sin(theta)],
    [0, 1, sin(phi)],
    [sin(theta), 0, cos(phi)*cos(theta)]]  *  [phi_dot, theta_dot, psi_dot]

w_mat = [
    [0, -w3, w2],
    [w3, 0, -w1],
    [-w2, w1, 0]]

// euler eq of motion to get the net moments
[Mc1, Mc2, Mc3] = I * w_dot + w_mat * I * w   // col vector
                = [L*(F2-F4), L*(F3-F1), M1-M2+M3-M4]  // L = arm length
                = u2

```

by combining these newton euler equation:
```rust
m*r_ddot = [0,0,-mg] + R * [0,0,u1]  // with u1 = F1+F2+F3+F4

I*[p_dot, q_dot, r_dot] = u2 - w * I * w

// 2D
[y_ddot, z_ddot, phi_ddot] = [0, -g, 0] + [[-1/m * (sin(phi)), 0],     * [u1, u2]
                                            [1/m * (cos(phi)), 0],
                                            [0, 1/Ixx]]
// 2D decomposed
y_ddot = -u1/m * sin(phi)
z_ddot = -g + u1/m * cos(phi)
phi_ddot = u2/Ixx

// these above eq are non linear, if we linearize at hover , so u1 = mg, phi=0, we have
y_ddot = -g * phi
z_ddot = -g + u1/m
phi_ddot = u2/Ixx
```
We want to follow trajectory T donoted by
- position vector r_T
- with velocity and acc r_dot_T and r_ddot_T

```rust
r_T = [y_T, z_T]

//error vector
err_pos = r_T - r    // with r= current position [y, z]
err_vel = r_dot_T - r_dot

//we want
(r_ddot_T - r_ddot_cmd)+ kv * err_vel + kp * err_pos = 0   // r_ddot_cmd is the commanded acceleration calculated by the controller

//altitude controller 2D
u1 = m*(g + z_ddot_des + kv(error_vel_z) + kp(error_pos_z))

// position controller 2D
phi_c = -y_ddot_c / g
      = -1/g * (y_ddot_des + kv(error_vel_y) + kp(error_pos_y))


// attitude control {inner loop} 2D
// phi_c and phi_c_dot are disered values computed by position controller
// phi and phi_dot are given by the drone
u2 = Ixx * (phi_ddot + kp(phi_c - phi) + kv(phi_c_dot - phi_dot))
// in simple trajectory Ixx = 1
```

Now for the 3D case
```rust
// we want to follow trajectory T
r_T = [x_T, y_T, z_T, psi]

err_pos = r_des - r
err_psi = psi_des - psi
// in a more compact way, we can construct an error vector
err = r_T - r
err_vel = r_dot_T - r_dot
// and we want 
(r_ddot_T - r_ddot_cmd)+ kv * err_vel + kp * err = 0   // r_ddot_cmd is the commanded acceleration calculated by the controller


// altitude
z_ddot_c = z_ddot_des + kv(error_vel_z) + kp(error_pos_z)
u1 = m*(g + z_ddot_c)

// position controller
y_ddot_c = y_ddot_des + kv(error_vel_y) + kp(error_pos_y)
x_ddot_c = x_ddot_des + kv(error_vel_x) + kp(error_pos_x)
phi_c = 1/g * (x_ddot_c * sin(psi_des) - y_ddot_c * cos(psi_des))
theta_c = 1/g * (x_ddot_c * cos(psi_des) + y_ddot_c * sin(psi_des))
psi_c = psi_des

// actual
x_ddot = g * (theta * cos(psi) + phi * sin(psi))
y_ddot = g * (theta * sin(psi) - phi * cos(psi))


// attitude : _c given by position controller and p,q,r, phi, theta, psi by the drone
u2 = [
    [kp(phi_c - phi) + kv(p_c - p)],
    [kp(theta_c - theta) + kv(q_c - q)],
    [kp(psi_c - psi) + kv(r_c - r)]
]

/* update state
        x_ddot = g * (theta * cos(psi) + phi * sin(psi))
        y_ddot = g * (theta * sin(psi) - phi * cos(psi))
        z_ddot = u1 / mass - 9.81;
        p_dot = (u2.0 - r * q * (izz - iyy)) / ixx
        q_dot = (u2.1 - r * self.p * (ixx - izz)) / iyy
        r_dot = (u2.2 - q * self.p * (iyy - ixx)) / izz

        

        we know that
        [p, q, r] = [[cos(theta), 0, -cos(phi)*sin(theta)],   *  [phi_dot, theta_dot, psi_dot]
                     [0, 1, sin(phi)],
                     [sin(theta), 0, cos(phi)*cos(theta)]]

        AND 
        c = a * b
        b = Inv(A) * c

        so,
        [phi_dot, theta_dot, psi_dot] = inv(...) * [p, q, r]
        
        phi_dot = (p*cos(theta))/(cos(theta)^2 + sin(theta)^2) + (r*sin(theta))/(cos(theta)^2 + sin(theta)^2)
        theta_dot = q - (r*cos(theta)*sin(phi))/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2) + (p*sin(phi)*sin(theta))/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2)
        psi_dot = (r*cos(theta))/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2) - (p*sin(theta))/(cos(phi)*cos(theta)^2 + cos(phi)*sin(theta)^2)

        */
```




### Control
