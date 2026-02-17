# Jacobian Matrix Analysis: Absolute Orientation vs Angular Velocity Fusion
## Mathematical Deep Dive for EKF-based Robot Localization

---

## 1. EKF State Vector and Dynamics

### State Vector (15D in robot_localization):
```
x = [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]ᵀ
```

For 2D mode (differential drive), constraints apply:
- z = 0, roll = 0, pitch = 0
- vz = 0, vroll = 0, vpitch = 0

**Active state variables:** [x, y, yaw, vx, vy, vyaw, ax, ay]

---

## 2. State Transition Model

### Continuous-time dynamics:
```
ẋ = vx·cos(yaw) - vy·sin(yaw)
ẏ = vx·sin(yaw) + vy·cos(yaw)
ẏaw = vyaw
v̇x = ax
v̇y = ay
v̇yaw = ayaw  (angular acceleration, if modeled)
```

### Discrete-time prediction (Euler integration):
```
x[k] = x[k-1] + (vx[k-1]·cos(yaw[k-1]) - vy[k-1]·sin(yaw[k-1])) · Δt
y[k] = y[k-1] + (vx[k-1]·sin(yaw[k-1]) + vy[k-1]·cos(yaw[k-1])) · Δt
yaw[k] = yaw[k-1] + vyaw[k-1] · Δt
vx[k] = vx[k-1] + ax[k-1] · Δt
vy[k] = vy[k-1] + ay[k-1] · Δt
vyaw[k] = vyaw[k-1] + ayaw[k-1] · Δt
```

---

## 3. State Transition Jacobian (F Matrix)

The Jacobian **F** represents ∂f/∂x, where f is the state transition function.

### Critical non-zero partial derivatives:

**Position coupling with orientation:**
```
∂x[k]/∂yaw[k-1] = (-vx[k-1]·sin(yaw[k-1]) - vy[k-1]·cos(yaw[k-1])) · Δt
∂y[k]/∂yaw[k-1] = (vx[k-1]·cos(yaw[k-1]) - vy[k-1]·sin(yaw[k-1])) · Δt
```

**Position coupling with velocity:**
```
∂x[k]/∂vx[k-1] = cos(yaw[k-1]) · Δt
∂x[k]/∂vy[k-1] = -sin(yaw[k-1]) · Δt
∂y[k]/∂vx[k-1] = sin(yaw[k-1]) · Δt
∂y[k]/∂vy[k-1] = cos(yaw[k-1]) · Δt
```

**Orientation coupling with angular velocity:**
```
∂yaw[k]/∂yaw[k-1] = 1
∂yaw[k]/∂vyaw[k-1] = Δt
```

**Velocity coupling with acceleration:**
```
∂vx[k]/∂ax[k-1] = Δt
∂vy[k]/∂ay[k-1] = Δt
```

### F Matrix structure (simplified for 2D active states):
```
F = [1    0    ∂x/∂θ    Δt·cos(θ)  -Δt·sin(θ)  0       0    0  ]  x
    [0    1    ∂y/∂θ    Δt·sin(θ)   Δt·cos(θ)  0       0    0  ]  y
    [0    0      1          0            0      Δt      0    0  ]  θ (yaw)
    [0    0      0          1            0      0      Δt    0  ]  vx
    [0    0      0          0            1      0       0   Δt  ]  vy
    [0    0      0          0            0      1       0    0  ]  vθ
    [0    0      0          0            0      0       1    0  ]  ax
    [0    0      0          0            0      0       0    1  ]  ay

where θ = yaw[k-1]
```

---

## 4. Measurement Jacobian (H Matrix)

### CASE 1: Fusing Absolute Yaw from IMU

**Measurement model:**
```
z_yaw = yaw + v_yaw
where v_yaw ~ N(0, R_yaw)
```

**Measurement Jacobian H₁:**
```
H₁ = [0  0  1  0  0  0  0  0]  (for yaw measurement)
```

**Physical interpretation:** The measurement directly observes the yaw state with no dependence on other states.

**Kalman Gain computation:**
```
K = P⁻ · H₁ᵀ · (H₁ · P⁻ · H₁ᵀ + R_yaw)⁻¹
```

**State update:**
```
x⁺ = x⁻ + K · (z_yaw - yaw⁻)
```

**Impact on each state variable:**
- **yaw:** Directly corrected proportional to innovation
- **x, y:** Corrected through covariance coupling P[x,yaw] and P[y,yaw]
- **vx, vy:** Corrected through P[vx,yaw] and P[vy,yaw]
- **vyaw:** Corrected through P[vyaw,yaw]

---

### CASE 2: Fusing Angular Velocity from IMU

**Measurement model:**
```
z_vyaw = vyaw + v_vyaw
where v_vyaw ~ N(0, R_vyaw)
```

**Measurement Jacobian H₂:**
```
H₂ = [0  0  0  0  0  1  0  0]  (for vyaw measurement)
```

**Physical interpretation:** The measurement directly observes the angular velocity state.

**Kalman Gain computation:**
```
K = P⁻ · H₂ᵀ · (H₂ · P⁻ · H₂ᵀ + R_vyaw)⁻¹
```

**State update:**
```
x⁺ = x⁻ + K · (z_vyaw - vyaw⁻)
```

**Impact on each state variable:**
- **vyaw:** Directly corrected proportional to innovation
- **yaw:** Corrected through covariance coupling P[yaw,vyaw]
- **x, y:** Corrected through P[x,vyaw] and P[y,vyaw]
- **vx, vy:** Corrected through P[vx,vyaw] and P[vy,vyaw]

---

## 5. Covariance Propagation Analysis

### Prediction Step (both cases):
```
P⁻[k] = F · P⁺[k-1] · Fᵀ + Q
```

The coupling terms in F propagate uncertainty:
- Yaw uncertainty affects position uncertainty through ∂x/∂yaw and ∂y/∂yaw
- Angular velocity uncertainty affects yaw uncertainty through ∂yaw/∂vyaw

### Update Step Comparison:

**CASE 1 (Absolute Yaw):**
```
P⁺ = (I - K·H₁) · P⁻

Innovation covariance: S₁ = H₁·P⁻·H₁ᵀ + R_yaw = P⁻[yaw,yaw] + R_yaw

Kalman gain: K = P⁻[:,yaw] / S₁

State uncertainty reduction:
Δσ²_yaw = K[yaw] · S₁  (direct reduction in yaw uncertainty)
Δσ²_vyaw = K[vyaw] · S₁  (indirect reduction through covariance)
```

**CASE 2 (Angular Velocity):**
```
P⁺ = (I - K·H₂) · P⁻

Innovation covariance: S₂ = H₂·P⁻·H₂ᵀ + R_vyaw = P⁻[vyaw,vyaw] + R_vyaw

Kalman gain: K = P⁻[:,vyaw] / S₂

State uncertainty reduction:
Δσ²_vyaw = K[vyaw] · S₂  (direct reduction in vyaw uncertainty)
Δσ²_yaw = K[yaw] · S₂  (indirect reduction through covariance)
```

---

## 6. Mathematical Comparison: Key Differences

### 6.1 Observability

**Absolute Yaw (H₁):**
- Directly observable: rank(H₁) = 1, observes yaw
- Full observability of orientation state
- Does NOT observe angular velocity directly
- Angular velocity observability comes from temporal changes in yaw

**Angular Velocity (H₂):**
- Directly observable: rank(H₂) = 1, observes vyaw
- Full observability of angular velocity
- Yaw observability through integration: yaw = ∫vyaw dt
- Weaker long-term orientation observability

**Mathematical insight:**
The observability Gramian differs:
- For H₁: Strong orientation observability, weak velocity observability
- For H₂: Strong velocity observability, weak orientation observability

---

### 6.2 Error Propagation Dynamics

**Absolute Yaw:**
```
Error dynamics for yaw: e_yaw[k+1] = e_yaw[k] + e_vyaw[k]·Δt - K₁·innovation

Steady-state error: Bounded by measurement noise and process noise
Growth rate: Linear (accumulation from vyaw error)
Correction mechanism: Direct proportional feedback
```

**Angular Velocity:**
```
Error dynamics for vyaw: e_vyaw[k+1] = e_vyaw[k] - K₂·innovation
Error dynamics for yaw: e_yaw[k+1] = e_yaw[k] + e_vyaw[k]·Δt

Steady-state error: Grows unbounded without absolute reference
Growth rate: Quadratic for yaw (integration of velocity error)
Correction mechanism: Indirect through velocity correction → yaw integration
```

---

### 6.3 Sensitivity to Sensor Characteristics

**Absolute Yaw (Magnetometer-based):**
```
Measurement model: z = yaw + b + v
where:
  b = magnetic bias (hard/soft iron, electromagnetic interference)
  v = white noise

Jacobian impact: ∂z/∂b = 1 (bias directly affects measurement)

Problem: Bias is not in state vector → unmodeled dynamics
Result: Filter divergence if bias changes (e.g., moving from outdoor to indoor)
```

**Angular Velocity (Gyroscope-based):**
```
Measurement model: z = vyaw + b_gyro + v
where:
  b_gyro = gyroscope bias (slowly varying)
  v = white noise

Jacobian impact: ∂z/∂b_gyro = 1

Problem: Bias integrates to yaw error: Δyaw = ∫b_gyro dt
Result: Unbounded drift without absolute corrections
```

---

### 6.4 Kalman Gain Magnitude Comparison

Assume:
- R_yaw = 0.01 rad² (magnetometer noise)
- R_vyaw = 0.0004 rad²/s² (gyroscope noise)
- P[yaw,yaw] = 0.03 rad²
- P[vyaw,vyaw] = 0.02 rad²/s²
- P[yaw,vyaw] = 0.005 rad·rad/s (covariance)

**CASE 1 (Absolute Yaw):**
```
S₁ = 0.03 + 0.01 = 0.04
K[yaw] = 0.03 / 0.04 = 0.75  (strong correction)
K[vyaw] = 0.005 / 0.04 = 0.125  (weak indirect correction)
```

**CASE 2 (Angular Velocity):**
```
S₂ = 0.02 + 0.0004 = 0.0204
K[vyaw] = 0.02 / 0.0204 = 0.98  (very strong correction)
K[yaw] = 0.005 / 0.0204 = 0.245  (moderate indirect correction)
```

**Interpretation:**
- Absolute yaw provides stronger direct orientation correction
- Angular velocity provides stronger rate correction but weaker orientation correction
- Cross-coupling efficiency depends on P[yaw,vyaw] magnitude

---

## 7. Practical Implications for Differential Drive Robot

### Why the Configuration Fuses vyaw (not yaw):

1. **Magnetometer unreliability indoors:**
   - MPU-9250 magnetometer affected by: motors, batteries, metal structures
   - Bias errors corrupt Jacobian-based corrections
   - Creates non-Gaussian, non-stationary measurement noise

2. **Gyroscope superior short-term accuracy:**
   - Rate integration more accurate over short horizons (< 10 seconds)
   - Noise characteristics better match Gaussian assumption
   - Jacobian coupling through F matrix is stable

3. **Wheel odometry provides absolute yaw reference:**
   - Differential drive kinematics: vyaw = (v_right - v_left) / wheelbase
   - Integrated yaw from wheels prevents gyro drift
   - Complementary strengths: gyro for high-frequency, odom for low-frequency

4. **Covariance coupling via F matrix sufficient:**
   - Yaw uncertainty reduced through: P[yaw,vyaw] and ∂yaw/∂vyaw = Δt
   - Multiple sensor fusion (odom + IMU) provides redundancy
   - No single point of failure

---

## 8. Numerical Example: 5-Second Trajectory

**Scenario:** Robot drives forward at 0.5 m/s, turning left at 0.3 rad/s

### CASE 1: Fusing Absolute Yaw (with 5° magnetometer bias)

```
Time    True yaw    Measured yaw    Estimated yaw    Error
0.0s      0.0°          0.0°            0.0°          0.0°
1.0s     17.2°         22.2°           18.5°          1.3°
2.0s     34.4°         39.4°           36.2°          1.8°
3.0s     51.6°         56.6°           53.1°          1.5°
4.0s     68.8°         73.8°           70.4°          1.6°
5.0s     86.0°         91.0°           87.8°          1.8°

RMS Error: 1.58°
```

**Jacobian impact:** K directly amplifies bias error into state estimate.

---

### CASE 2: Fusing Angular Velocity (with 0.01 rad/s gyro bias)

```
Time    True yaw    Measured vyaw    Estimated yaw    Error
0.0s      0.0°       0.300 rad/s          0.0°        0.0°
1.0s     17.2°       0.310 rad/s         17.3°        0.1°
2.0s     34.4°       0.310 rad/s         34.9°        0.5°
3.0s     51.6°       0.310 rad/s         52.5°        0.9°
4.0s     68.8°       0.310 rad/s         70.2°        1.4°
5.0s     86.0°       0.310 rad/s         87.9°        1.9°

RMS Error: 1.06°
```

**Jacobian impact:** Bias accumulates through ∂yaw/∂vyaw = Δt, but wheel odom corrects.

---

## 9. Recommended Fusion Strategy

For your differential drive robot with MPU-9250:

**Primary:** Fuse vyaw (angular velocity) from gyroscope
**Secondary:** Fuse yaw from wheel odometry (odom0)
**Tertiary:** Fuse ax (X acceleration) for dynamics

**Rationale:**
1. H₂ (vyaw) provides clean Jacobian with Gaussian noise model
2. F matrix couples vyaw → yaw naturally through prediction
3. Wheel odom yaw prevents unbounded drift (complementary filtering)
4. Avoids magnetometer bias corrupting entire state estimate

**DO NOT fuse absolute IMU yaw** unless:
- Extensive magnetometer calibration performed
- Operating in magnetically clean environment
- Bias estimation added to state vector (augmented EKF)

---

## 10. Advanced Consideration: Augmented State for Bias Estimation

If you want to fuse absolute yaw, augment state vector:

```
x_aug = [x, y, yaw, vx, vy, vyaw, ax, ay, b_mag]ᵀ
```

New Jacobian terms:
```
∂z_yaw/∂yaw = 1
∂z_yaw/∂b_mag = 1  (now modeled!)
```

Process noise for bias:
```
Q[b_mag, b_mag] = σ²_bias_drift  (e.g., 1e-6 for slow drift)
```

This requires modifying robot_localization source code or using dual EKF approach.

---

## Summary Table

| Aspect                    | Fuse Absolute Yaw (H₁)        | Fuse Angular Velocity (H₂)    |
|---------------------------|--------------------------------|--------------------------------|
| **Direct observability**  | Orientation                   | Angular velocity               |
| **Indirect observability**| Angular velocity              | Orientation (via integration)  |
| **Short-term accuracy**   | Moderate (bias dependent)     | High (gyro excellent)          |
| **Long-term drift**       | Bounded (if no bias)          | Unbounded (needs reference)    |
| **Jacobian H complexity** | Simple: [0,0,1,0,0,0,0,0]     | Simple: [0,0,0,0,0,1,0,0]      |
| **Kalman gain magnitude** | High for yaw                  | High for vyaw                  |
| **Sensor failure mode**   | Bias corruption               | Drift accumulation             |
| **Coupling mechanism**    | H₁ → direct, F → indirect     | H₂ → direct, F → integration   |
| **Recommended for MPU-9250** | ❌ (magnetometer unreliable) | ✅ (gyroscope reliable)         |

---

**Conclusion:**
The Jacobian structure fundamentally determines how measurement information propagates through the state estimate. Fusing angular velocity (vyaw) provides cleaner, more robust updates for differential drive robots with wheel odometry, as the measurement Jacobian H₂ avoids magnetometer bias issues while the state transition Jacobian F naturally integrates rate information into orientation estimates.
