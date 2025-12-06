# Old 3DOF Hydrofoil Boat Model

This folder contains the first-generation hydrofoil boat dynamics model.
It implements a very simplified 3DOF rigid-body system written initially in symbolic form and later converted into a MATLAB ODE function.
It is mainly intended for prototyping and gaining intuition, not for accurate prediction.

## 1. Model Overview

The state vector is:
```
x = [ zW; zWdot; phi; phidot; theta; thetadot ]
```

where:

- `zW` is heave (vertical position in world frame, NED convention)
- `phi` is roll about body x-axis
- `theta` is pitch about body y-axis
- angle derivatives are treated as angular velocities (approximation)

The model includes:

- Gravity
- Hydrofoil lift & drag forces
- Simple buoyancy (scalar, vertical only)
- Roll & pitch torques from hydrofoil forces
- Thrust as a body-frame +x force

All geometry and aerodynamic data come from `boat_model_parameters_3dof.m`.

## 2. Major Simplifications (Important)

This model is intentionally simplistic. The key approximations are:

### (1) Only 3 Degrees of Freedom

- Heave (z), roll (φ), pitch (θ)
- No surge, sway, or yaw
- Forward speed $V_W$ is an input, not produced by physics

### (2) Rotational dynamics use a naïve torque/inertia relation

**Roll:**
$$\ddot{\phi} = \frac{\tau_x}{I_x}$$

**Pitch:**
$$\ddot{\theta} = \frac{\tau_y}{I_y}$$

This ignores:

- Gyroscopic coupling
- Coriolis terms
- Non-inertial effects of using a rotating body frame
- Cross-axis inertia effects

This is valid only for small angles and small angular rates.

### (3) Lift and drag direction assumptions

Lift is assumed to act along:
$$\mathbf{e}_{\text{lift},W} = [0, \sin\phi, -\cos\phi]$$

Drag acts purely opposite $x_W$:
$$\mathbf{e}_{\text{drag},W} = [-1, 0, 0]$$

This ignores:

- Pitch-induced rotation of the lift vector
- Induced velocities
- Hydrodynamic inflow angles

### (4) Vertical dynamics ignore added mass and damping

**Heave equation:**
$$\ddot{z}_W = \frac{mg - F_{z,\text{foils}} - F_{\text{buoy}}}{m}$$

There is:

- No heave damping
- No added-mass effects
- No wave radiation forces

### (5) Foil exit model is binary

If foil depth in world frame is above water (z < 0):
→ lift = drag = 0 immediately.

No smooth transition, no partial immersion physics.

### (6) Buoyancy is linearized and overly simple

Buoyancy depends only on heave and not on roll or pitch, even though roll/pitch should shift the center of buoyancy.

Defined in `buoyancy_force()` in the dynamics file.

## 3. Equations Implemented

### 3.1 Heave dynamics

From the ODE:

$$\dot{z}_W = v_z$$

$$\ddot{z}_W = \frac{mg - F_{z,\text{foils}} - F_{\text{buoy}}}{m}$$

Here $F_{z,\text{foils}}$ is the transformed sum of forces from all hydrofoils.

### 3.2 Roll dynamics

$$\ddot{\phi} = \frac{\tau_x}{I_x}$$

Where:
$$\tau_x = \sum_i (\mathbf{r}_i \times \mathbf{F}_i)_x$$

Lift asymmetry between left and right front foils drives roll.

### 3.3 Pitch dynamics

$$\ddot{\theta} = \frac{\tau_y}{I_y}$$

Pitch torque includes:

- Front foil lift & drag
- Rear foil lift & drag
- Thrust offset torque

as expressed in symbolic form and implemented numerically in the dynamics file.

### 3.4 Lift/drag magnitudes

$$F_L = \frac{1}{2}\rho S V^2 C_L(\alpha)$$

$$F_D = \frac{1}{2}\rho S V^2 C_D(\alpha)$$

with:
$$\alpha = \theta + \alpha_{\text{actuator}}$$

Lookup tables for $C_L$/$C_D$ are loaded from Eppler E874 data (in `boat_model_parameters_3dof.m`).

## 4. Limitations of This Old Model

This model is suitable for conceptual understanding but **NOT** for:

- Stability analysis
- Controller design
- Real-world predictive simulation

### Main missing elements:

| Missing Element | Consequence |
|---|---|
| Body-frame rotational dynamics (Coriolis, gyroscopics) | Wrong roll/pitch acceleration for large angles/velocities |
| Added mass | Underestimates inertia in heave & pitch |
| Hydrodynamic damping | Unlimited oscillations possible |
| Yaw & surge | Cannot simulate turning nor acceleration behavior |
| Hydrostatic restoring moments | Boat may not self-right realistically |
| Smooth foil exit | Discontinuous dynamics when foils leave surface |

## 5. What This Model Is Good For

Despite limitations, this simplified system is very useful for early-stage reasoning:

### ✓ Understanding basic coupling

Effects of:

- Lift geometry
- Foil placement
- Thrust offset

become immediately visible.

### ✓ Rapid prototyping

The model runs extremely fast and is simple enough to embed inside Simulink without stiffness issues.

### ✓ Sensitivity studies

You can easily test:

- Influence of $C_L$/$C_D$ curves
- Foil area changes
- Front/rear lift ratios

### ✓ Educational value

It provides a clean entry point into:

- Coordinate frames
- Force transformations
- Basic hydrofoil aerodynamics

## 6. Natural Next Steps (Where the new model will go)

The limitations point directly to the next layer of modeling:

- **Switch rotational dynamics to Newton–Euler in body frame**
	$$I\dot{\omega} + \omega \times (I\omega) = \tau$$

- **Introduce body angular rates** (p, q, r) and full attitude (φ, θ, ψ)

- **Add surge equation** to make boat forward motion physical

- **Implement hydrostatic restoring moments** for roll/pitch stability

- **Add damping and added mass terms**

- **Improve foil immersion model** → smooth transition from submerged to ventilated

These changes form the foundation for the new, improved model you will develop next.
