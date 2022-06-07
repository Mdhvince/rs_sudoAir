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

### Control
