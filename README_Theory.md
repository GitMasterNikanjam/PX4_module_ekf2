# PX4 EKF2 — Theory and Architecture

> **Purpose**
>
> This document explains the mathematical theory and software architecture behind PX4's
> `EKF2` navigation estimator, with special attention to the supplied `EKF2.cpp`,
> `EKF2.hpp`, `EKF2Selector.cpp`, and `EKF2Selector.hpp` files.
>
> The four supplied files are primarily the **PX4 integration, sensor I/O, publication,
> parameter, and multi-EKF selection layer**. The detailed covariance and observation
> Jacobians are implemented/generated inside the core `EKF/` library. Therefore this
> document separates:
>
> 1. behavior that is directly visible in the supplied files, and
> 2. the underlying EKF mathematics documented by current PX4 source and its symbolic
>    derivation.

---


## Viewing the Equations in VS Code

This file uses VS Code-compatible Markdown math syntax:

```text
Inline equation:  $x = a + b$

Display equation:

$$
x_{k+1} = F_k x_k + G_k w_k
$$
```

To view the rendered equations in VS Code:

1. Open `README_Theory.md`.
2. Press **Ctrl+Shift+V** to open **Markdown Preview**.
3. For side-by-side editing and preview, press **Ctrl+K**, then **V**.

Use the Markdown Preview instead of reading the raw `.md` editor when you want the
formulas rendered.

---

## 1. What EKF2 Does

`EKF2` is PX4's tightly coupled inertial navigation estimator. Its job is to combine a
high-rate IMU propagation model with slower and delayed aiding sensors in order to
estimate quantities needed by the flight-control system.

The estimator can provide, depending on build options and enabled aiding sources:

- attitude;
- local NED velocity;
- local and global position;
- gyro bias;
- accelerometer bias;
- earth magnetic field;
- body-frame magnetic disturbance/bias;
- horizontal wind;
- terrain vertical position / height above ground;
- uncertainty/covariance information;
- innovation diagnostics;
- estimator health and fault flags.

The estimator is **not just a low-pass filter**. It maintains a nonlinear navigation
state and a covariance matrix describing uncertainty and cross-correlation between
state errors.

A useful high-level view is

```text
                               delayed aiding measurements
 GNSS ----------------------\
 barometer ------------------\
 magnetometer ----------------\
 range finder -----------------\
 optical flow -------------------> Core error-state EKF
 external vision --------------/      at fusion horizon
 airspeed ---------------------/
 sideslip / drag -------------/
 ranging beacon -------------/
                                     |
                                     | corrected delayed state
                                     v
 IMU ---> inertial propagation ---> Output predictor ---> current-time attitude/
          and covariance              |                  velocity/position
                                      |
                                      +--> diagnostics, innovations,
                                           reset information, health

 Multiple EKF instances
        |
        v
 EKF2Selector ---> chooses primary estimator ---> vehicle_* system topics
```

---

## 2. Software Layers

The supplied source makes an important architectural distinction.

### 2.1 `EKF2`

`EKF2` is the PX4 module wrapper around an internal object:

```cpp
Ekf _ekf;
```

The wrapper is responsible for tasks such as:

- subscribing to PX4 uORB sensor topics;
- converting topic messages into EKF sample structures;
- loading and validating `EKF2_*` parameters;
- pushing measurements into `_ekf`;
- calling `_ekf.update()`;
- publishing attitude, local position, global position, odometry, biases, status,
  innovations, and aid-source diagnostics;
- handling sensor calibration changes;
- tracking reset counters;
- supporting replay mode;
- supporting multiple estimator instances.

The core nonlinear estimation mathematics is therefore conceptually below this layer:

```text
PX4/uORB world
     |
     v
  EKF2.cpp
     |
     v
  Ekf class
     |
     v
state propagation + covariance propagation + aiding controllers + measurement fusion
```

### 2.2 `EKF2Selector`

When PX4 runs several EKF instances, `EKF2Selector` compares them and republishes the
selected instance as the system-level estimate.

Its job is estimator **redundancy and fault isolation**, not Kalman filtering itself.

---

## 3. Coordinate Frames

Understanding the frames is essential.

### 3.1 Local navigation frame: NED

PX4 navigation convention is normally

```text
X = North
Y = East
Z = Down
```

So altitude increasing upward corresponds to local `z` becoming more negative.

This is why the wrapper contains relationships such as

```text
global altitude change = - local Down-position change
```

### 3.2 Body frame: FRD

The aircraft body frame is conventionally

```text
X = Forward
Y = Right
Z = Down
```

### 3.3 Quaternion convention

In the current core derivation, the nominal quaternion is used such that its rotation
matrix transforms a body-frame vector into the local earth/NED frame:


$$
\mathbf v^n = \mathbf R(q)\mathbf v^b
$$


and therefore


$$
\mathbf v^b = \mathbf R(q)^T\mathbf v^n
$$


In code this appears through concepts such as

```text
R_to_earth
R_to_body = R_to_earth^{-1}
```

Always check the exact PX4 message/frame definition when interfacing an external
system. A mathematically correct estimator can still fail badly if NED, ENU, FRD,
FLU, or quaternion direction conventions are mixed.

---

## 4. Why EKF2 Is an Error-State EKF

A normal EKF could place a four-element quaternion directly in the covariance state.
That is awkward because a unit quaternion has four stored values but only three
rotational degrees of freedom.

Current PX4 EKF2 uses an **error-state formulation**.

The estimator keeps a nonlinear **nominal state**, while the covariance describes a
small perturbation around that nominal state.

Conceptually,


$$
x_{\text{true}} = x_{\text{nominal}} \boxplus \delta x
$$


where `boxplus` means ordinary addition for Euclidean states and a small rotation
composition for attitude.

For attitude, let the small attitude error be


$$
\delta\boldsymbol\theta =
\begin{bmatrix}
\delta\theta_x & \delta\theta_y & \delta\theta_z
\end{bmatrix}^T
$$


For small angles the corresponding error quaternion is approximately


$$
\delta q \approx
\begin{bmatrix}
1 \\
\frac{1}{2}\delta\boldsymbol\theta
\end{bmatrix}
$$


depending on storage ordering/convention.

This allows attitude uncertainty to be represented with **three** variables instead
of four constrained quaternion components.

Benefits include:

- physically minimal attitude error coordinates;
- better numerical conditioning;
- covariance defined in the tangent space of $SO(3)$;
- clean injection of small corrections into the nominal quaternion;
- easier symbolic derivation of Jacobians.

---

## 5. Nominal State and Error State

The current PX4 symbolic derivation defines a nominal state containing the following
logical groups:


$$
x =
\left[
q,\;
v,\;
p,\;
b_g,\;
b_a,\;
B_I,\;
B_B,\;
w,\;
h_T
\right]
$$


where:

| Symbol | Meaning |
|---|---|
| $q$ | nominal attitude quaternion |
| $v$ | NED velocity |
| $p$ | position |
| $b_g$ | gyro bias |
| $b_a$ | accelerometer bias |
| $B_I$ | earth magnetic field in navigation frame |
| $B_B$ | body magnetic field bias/disturbance |
| $w$ | horizontal wind velocity |
| $h_T$ | terrain vertical-position state |

The corresponding error state is conceptually


$$
\delta x =
\left[
\delta\theta,\;
\delta v,\;
\delta p,\;
\delta b_g,\;
\delta b_a,\;
\delta B_I,\;
\delta B_B,\;
\delta w,\;
\delta h_T
\right]^T
$$


The nominal quaternion requires four stored numbers, but its error requires only the
three-dimensional $\delta\theta$.

### Important version note

Do **not** hard-code a state index map from an old PX4 article into analysis software
without checking the exact PX4 revision.

The supplied wrapper publishes

```cpp
const auto state_vector = _ekf.state().vector();
state_vector.copyTo(states.states);
states.n_states = state_vector.size();

_ekf.covariances_diagonal().copyTo(states.covariances);
```

so the authoritative layout for a particular build is the generated/core EKF state
definition used by that build.

Older PX4 documentation often shows a 24-value public state layout without the
separate terrain state, while the current symbolic error-state derivation includes
terrain as a generated state. Treat the source revision, generated `state.h`, and
uORB message definition as the source of truth.

---

## 6. IMU as the Process Model

The IMU is special: it is primarily used to **propagate** the navigation state.

Typical aiding sensors produce observations:

```text
GNSS -> position/velocity observation
baro -> height observation
mag -> magnetic-field/heading observation
...
```

The IMU instead tells the estimator how the vehicle state evolves between
observations.

The supplied wrapper constructs an `imuSample` with:

- integrated delta angle;
- delta-angle integration time;
- integrated delta velocity;
- delta-velocity integration time;
- accelerometer clipping flags;
- timestamp.

Then it calls

```cpp
_ekf.setIMUData(imu_sample_new);
```

followed later by

```cpp
_ekf.update();
```

---

## 7. Nominal Inertial Propagation

Let

- $\Delta\theta_m$ = measured integrated body rotation;
- $\Delta v_m$ = measured integrated specific force;
- $b_g$ = gyro bias;
- $b_a$ = accelerometer bias.

The bias-corrected quantities are conceptually


$$
\Delta\theta = \Delta\theta_m - b_g \Delta t
$$


$$
\Delta v^b = \Delta v_m - b_a \Delta t
$$


The exact implementation may use integrated-bias states or rate-bias states depending
on PX4 revision; the principle is the same.

### 7.1 Attitude propagation

The quaternion is propagated by composing it with the incremental body rotation:


$$
q_{k+1} =
q_k \otimes \delta q(\Delta\theta)
$$


and is normalized after the update.

At navigation quality, earth rotation can also be compensated.

### 7.2 Velocity propagation

The IMU delta velocity is measured in body axes. It is rotated into NED:


$$
\Delta v^n =
R(q)\Delta v^b
$$


Gravity is then added with the appropriate NED sign convention.

A discrete form is


$$
v_{k+1}
=
v_k +
\left[
R(q_k)(a_m-b_a) + g^n
\right]\Delta t
$$


where in NED


$$
g^n \approx
\begin{bmatrix}
0\\
0\\
+g
\end{bmatrix}
$$


because Down is positive.

### 7.3 Position propagation

A simplified discrete model is


$$
p_{k+1} = p_k + v_k\Delta t
$$


or a higher quality integration using both old/new velocity.

PX4's modern implementation also distinguishes global geodetic navigation from local
NED output and accounts for earth geometry where appropriate.

---

## 8. Bias States

MEMS gyros and accelerometers do not measure perfectly:


$$
\omega_m = \omega_{\text{true}} + b_g + n_g
$$


$$
a_m = a_{\text{true}} + b_a + n_a
$$


Even a small bias produces rapidly growing inertial-navigation errors.

### 8.1 Gyro bias

A yaw-rate bias causes attitude drift. Once attitude is wrong, gravity can project
incorrectly into horizontal axes and create false acceleration.

### 8.2 Accelerometer bias

An accelerometer bias integrates once into velocity error and twice into position
error.

### 8.3 Bias process model

Biases are usually modeled approximately as random walks:


$$
b_{g,k+1} = b_{g,k} + w_{bg}
$$


$$
b_{a,k+1} = b_{a,k} + w_{ba}
$$


with process-noise strengths configured through parameters such as the gyro-bias and
accelerometer-bias process-noise parameters.

Larger bias process noise allows faster adaptation but increases estimator noise and
can weaken fault discrimination.

### 8.4 Bias learning inhibition

The source exposes parameters and logic that can inhibit accelerometer/gyro bias
learning during conditions where the bias is poorly observable or the IMU is highly
dynamic.

This is important: **a Kalman filter cannot estimate a state reliably simply because
the state exists in the vector**. The motion and measurements must make that state
observable.

---

## 9. Covariance Matrix

The covariance matrix is


$$
P = E[\delta x \delta x^T]
$$


Its diagonal entries are state-error variances. Off-diagonal terms encode
cross-correlations.

For example, an attitude error can correlate with velocity error because incorrect
attitude rotates measured acceleration in the wrong direction.

That correlation is what lets a GNSS velocity measurement indirectly correct
attitude and IMU bias.

---

## 10. Covariance Prediction

The nonlinear error dynamics are linearized locally:


$$
\delta x_{k+1}
\approx
F_k \delta x_k + G_k w_k
$$


with

- $F_k$: state-transition Jacobian;
- $G_k$: process-noise mapping;
- $w_k$: process noise.

Then


$$
P_{k+1}^{-}
=
F_k P_k^{+} F_k^T
+
G_k Q_k G_k^T
$$


where $Q_k$ contains assumed IMU/process-noise variances.

PX4's current derivation builds these equations symbolically using **SymForce** and
generates optimized C++.

This matters for embedded systems because directly evaluating generic matrix
expressions for every sensor at high rate would waste CPU and flash.

---

## 11. The Generic Measurement Update

For a sensor observation $z$,


$$
z = h(x) + v
$$


where

- $h(x)$ predicts what the sensor should measure;
- $v$ is measurement noise;
- $R$ is the covariance of that noise.

### 11.1 Predicted measurement


$$
\hat z = h(\hat x^-)
$$


### 11.2 Innovation

PX4 commonly uses the convention


$$
\nu = \hat z - z
$$


that is,

```text
innovation = predicted observation - measured observation
```

Many textbooks use the opposite sign $z-\hat z$, so be careful when comparing
equations. The Kalman correction sign must match the innovation convention.

### 11.3 Observation Jacobian

The measurement model is linearized with


$$
H =
\frac{\partial h}{\partial \delta x}
$$


Because attitude is an error-state rotation, PX4's symbolic derivation explicitly
maps from quaternion storage space into the tangent/error space when building $H$.

### 11.4 Innovation variance

For a scalar observation,


$$
S = HPH^T + R
$$


For a vector observation the same equation produces a matrix $S$.

`S` answers:

> Given my current state uncertainty and the sensor noise, how much innovation should
> I statistically expect?

### 11.5 Kalman gain


$$
K = PH^T S^{-1}
$$


The gain becomes larger when the filter is uncertain and the sensor is trusted, and
smaller when the sensor is noisy or the state is already well known.

### 11.6 Error injection

The EKF computes an error-state correction and injects it into the nominal state.

For Euclidean states this behaves like ordinary addition/subtraction according to the
innovation sign convention.

For attitude the correction is applied as a small rotation to the nominal quaternion,
followed by normalization.

After injection, the error state is conceptually reset to zero while its updated
covariance remains.

---

## 12. Covariance Measurement Update and Numerical Stability

The basic covariance update is often written


$$
P^+ = (I-KH)P^-
$$


but this compact form is sensitive to floating-point roundoff.

PX4 documentation states that the modern EKF uses a **Joseph stabilized covariance
update**:


$$
P^+
=
(I-KH)P^-(I-KH)^T
+
KRK^T
$$


This costs more arithmetic but helps preserve:

- covariance symmetry;
- non-negative variances;
- numerical consistency.

PX4 also performs covariance sanity checks and constrains invalid or excessive
variance values.

This is particularly important because PX4 performs the estimator calculations in
single-precision floating point on embedded processors.

---

## 13. Innovation Gating

A measurement should not always be fused just because it exists.

Suppose a sensor says something wildly inconsistent with the predicted state. Fusing
it blindly can corrupt the filter.

EKF2 applies statistical gates.

For a scalar measurement, a useful conceptual normalized test is


$$
T =
\frac{\nu^2}
     {g^2 S}
$$


where

- $\nu$ = innovation;
- $S$ = innovation variance;
- $g$ = configured gate size in standard deviations.

Interpretation:

```text
T < 1   measurement is inside the statistical gate
T = 1   measurement is on the gate boundary
T > 1   measurement is inconsistent / normally rejected
```

PX4 publishes innovation test ratios for many aiding sources. The supplied wrapper
publishes dedicated topics for:

- innovations;
- innovation variances;
- innovation test ratios;
- per-aid-source status.

Do not confuse:

```text
innovation             -> signed residual
innovation variance    -> expected residual variance
test ratio              -> normalized consistency metric
```

A large innovation is not automatically bad if the predicted uncertainty is also
large. A relatively modest innovation can be bad if the estimator and sensor both
claim very high precision.

---

## 14. Why Sensor Noise Tuning Changes Behavior

If the measurement covariance $R$ is reduced:


$$
S = HPH^T + R
$$


usually becomes smaller and the Kalman gain tends to increase.

The filter then:

- trusts that sensor more;
- corrects toward it more aggressively;
- rejects disagreements more easily if the gate is not changed;
- can become sensitive to unmodeled sensor error.

If $R$ is increased:

- the measurement has less influence;
- the filter becomes smoother;
- real corrections take longer;
- sensor inconsistency is less likely to exceed the statistical gate.

Correct tuning therefore means modeling **real sensor uncertainty**, not merely
choosing values that make plots look smooth.

---

## 15. Delayed Fusion Horizon

One of the most important EKF2 design ideas is delayed measurement fusion.

Different sensors have different latencies:

```text
IMU              very low latency
magnetometer     modest latency
barometer        modest latency
GNSS             often larger latency
external vision  transport + compute latency
optical flow     integration-window latency
```

If a GNSS observation measured the vehicle 150 ms ago but is fused against the
current state, aggressive flight produces a systematic innovation error.

EKF2 instead maintains a **delayed fusion time horizon**.

Conceptually:

```text
current time -------------------------------------------->
                      |
                      | current output
                      v
      [buffered IMU history]
  -------------------+------------------------------------
          ^
          |
          fusion horizon
          |
          +-- delayed GNSS
          +-- delayed vision
          +-- delayed baro
          +-- delayed flow
```

Each sensor observation is buffered and fused when the estimator reaches the
corresponding measurement time.

The supplied wrapper contains delay parameters for several sensors and verifies that

```text
EKF2_DELAY_MAX >= maximum configured sensor delay
```

so the internal history is long enough.

---

## 16. Optical-Flow Timestamping Example

The wrapper shows a particularly instructive timing detail.

Optical flow is integrated over a time interval, so the physically correct
measurement time is near the **middle of the integration interval**, not simply the
message-arrival time.

The wrapper therefore forms a timestamp conceptually like

```text
sample_time =
    timestamp_sample - integration_timespan / 2
```

This is a good example of why estimator timing is part of the measurement model.

---

## 17. Current-Time Output Predictor

The delayed EKF state is statistically convenient for sensor fusion, but a flight
controller cannot tolerate an estimate that is intentionally delayed by tens or
hundreds of milliseconds.

Therefore EKF2 has a second stage: the **output predictor**.

Its job is to propagate the corrected delayed state forward to current time using
buffered/current IMU data.

Conceptually:

```text
                   corrections
 delayed EKF ------------------------+
     |                               |
     |                               v
     +---- historical IMU ----> output predictor ----> current-time output
```

The predictor must satisfy two competing goals:

1. very low latency;
2. long-term agreement with the delayed EKF solution.

PX4 uses configurable position and velocity correction time constants, exposed in the
wrapper through parameters such as

```text
EKF2_TAU_POS
EKF2_TAU_VEL
```

Reducing these constants makes the predictor track EKF corrections faster, but passes
more correction noise into control outputs.

The wrapper deliberately publishes attitude after the EKF update using the
output-predictor quaternion.

---

## 18. Sensor Lever Arms

Sensors may not be physically located at the IMU/body origin.

If the vehicle rotates, a sensor offset from the rotation center has additional
velocity:


$$
v_s = v_o + \omega \times r
$$


where

- $r$ = sensor lever arm;
- $\omega$ = body angular rate.

Ignoring this can create measurement residuals during rotation.

The supplied wrapper exposes body-frame position parameters for:

- IMU;
- external-vision sensor;
- optical-flow sensor;
- range sensor;
- GNSS antenna offset through the GNSS sample.

The core estimator can therefore correct observations to a common reference point.

---

# Sensor-Specific Theory

## 19. GNSS Position Fusion

GNSS provides global position. EKF2 maps global information into its navigation
representation and uses it to constrain position drift.

A simplified local horizontal position observation is


$$
z_p =
\begin{bmatrix}
p_N\\
p_E
\end{bmatrix}
+ v
$$


with


$$
H_p =
\begin{bmatrix}
0 & \cdots & I_{2\times2} & \cdots
\end{bmatrix}
$$


For direct-state position fusion the residual is essentially


$$
\nu_p = \hat p - z_p
$$


The position measurement also indirectly corrects velocity, attitude, and bias states
through the off-diagonal covariance terms.

### 19.1 GNSS quality checks

Before normal fusion, EKF2 checks receiver quality metrics such as:

- fix type;
- satellite count;
- position accuracy;
- vertical accuracy;
- speed accuracy;
- PDOP;
- horizontal drift;
- vertical drift;
- spoofing status.

The wrapper publishes these pass/fail results through `estimator_gps_status`.

This illustrates two different rejection layers:

```text
GNSS receiver-quality checks
             |
             v
 measurement admitted to EKF?
             |
             v
 innovation statistical gate
```

A receiver can have a nominal fix but still produce a statistically inconsistent
measurement.

---

## 20. GNSS Velocity Fusion

GNSS velocity is especially valuable because it directly constrains inertial drift.


$$
z_v =
\begin{bmatrix}
v_N\\
v_E\\
v_D
\end{bmatrix}
+ v
$$


During horizontal acceleration, a yaw error causes predicted acceleration and
velocity to point in the wrong horizontal direction. GNSS velocity innovations can
therefore make yaw observable even without continuous magnetometer aiding.

This is one reason a tightly coupled navigation EKF can estimate attitude better than
an attitude-only estimator.

---

## 21. Global Origin and Local Position

The wrapper publishes both:

- global WGS84-style position;
- local NED position.

When a valid global origin exists, local position includes the reference:

```text
ref_lat
ref_lon
ref_alt
ref_timestamp
```

Global and local representations are therefore two views of the same navigation
solution, not independent filters.

The wrapper also supports explicitly setting the global origin through a vehicle
command.

---

## 22. Ellipsoid Height and MSL Height

GNSS receivers can report height relative to the reference ellipsoid, while flight
interfaces often use altitude above mean sea level.

The wrapper estimates/filter the geoid-height offset:


$$
h_{\text{geoid}}
=
h_{\text{ellipsoid}}
-
h_{\text{AMSL}}
$$


and converts using


$$
h_{\text{AMSL}}
=
h_{\text{ellipsoid}}
-
h_{\text{geoid}}
$$


$$
h_{\text{ellipsoid}}
=
h_{\text{AMSL}}
+
h_{\text{geoid}}
$$


This prevents a datum mismatch from appearing as a large vertical estimator error.

---

## 23. Barometric Height Fusion

A barometer provides pressure altitude, not a direct geometric position.

A simplified vertical observation can be represented as


$$
z_h = h + b_{\text{baro}} + v
$$


where $b_{\text{baro}}$ accounts for slowly varying pressure-related offset.

The supplied wrapper exposes:

- baro measurement noise;
- baro gate;
- delay;
- ground-effect parameters;
- barometer bias diagnostics.

### 23.1 Barometer bias estimation

If another height source provides a long-term reference, barometer drift can be
estimated separately.

This prevents slow atmospheric pressure change from being interpreted as real
aircraft climb/descent.

### 23.2 Static pressure position error

Vehicle aerodynamics can perturb the static pressure measured by the barometer.

With wind estimation active, PX4 can compensate pressure error as a function of
relative airflow and body direction using configurable coefficients.

---

## 24. Height Reference vs Height Aiding

Multiple sensors may simultaneously constrain vertical position:

- GNSS;
- barometer;
- range;
- external vision.

One source is treated as the long-term **height reference** while others can still
aid the estimate.

This distinction is important:

```text
reference source  -> defines long-term vertical datum
aiding source     -> adds short/medium-term information and robustness
```

If two height sensors have slowly drifting but different datums, a bias estimator can
align them rather than forcing the main navigation state to jump.

---

## 25. Magnetometer Fusion

The body magnetometer approximately measures


$$
z_m =
R(q)^T B_I + B_B + v
$$


where

- $B_I$ = earth magnetic field in navigation coordinates;
- $B_B$ = body-frame magnetic bias/disturbance;
- $R(q)^T$ rotates earth field into body axes.

This observation depends on:

- attitude;
- earth field;
- body magnetic bias.

Therefore a 3-axis magnetic update can correct more than yaw.

### 25.1 Heading-only fusion

A heading observation can instead constrain only the yaw-related degree of freedom.

This reduces the risk of a distorted field corrupting roll/pitch, but gives up some
information available in the full 3-D magnetic vector.

### 25.2 Observability

Body magnetic bias becomes better observable when the vehicle rotates because the
earth field changes direction in body coordinates while a body-fixed bias does not.

---

## 26. Gravity-Vector Fusion

When the accelerometer mostly measures gravity, the normalized body acceleration
direction should agree with the gravity direction predicted from attitude:


$$
\hat z_g =
R(q)^T
\begin{bmatrix}
0\\
0\\
-1
\end{bmatrix}
$$


using the sign convention of the normalized gravity observation.

This can provide attitude information, but only when translational acceleration is
small enough that the accelerometer is a good gravity-direction sensor.

That is why gravity fusion must be conditionally enabled rather than blindly applied
during aggressive maneuvers.

---

## 27. External Vision Fusion

External vision can supply:

- horizontal position;
- height;
- velocity;
- yaw/orientation.

The wrapper accepts several frame choices and validates that incoming position,
velocity, quaternion, and variance data are finite and consistent.

### 27.1 Frames

External systems frequently use ENU/FLU conventions. PX4 EKF2 expects specific NED/FRD
or explicitly identified frames.

Frame conversion errors often look like EKF instability even though the Kalman
mathematics is correct.

### 27.2 Observation covariance

If the vision system reports valid variances, the wrapper can use them, while
enforcing configured minimum noise.

Conceptually:


$$
R_{\text{used}}
=
\max(R_{\text{reported}}, R_{\text{minimum}})
$$


per axis.

This prevents an external estimator from claiming unrealistically tiny uncertainty
and dominating the flight estimator.

### 27.3 Vision reset counter

A VIO/SLAM system may internally relocalize and jump its coordinate frame. The
`reset_counter` lets EKF2 distinguish a legitimate upstream reset from normal motion.

---

## 28. Optical Flow Fusion

Optical flow measures angular image motion, approximately related to translational
velocity divided by distance to the observed surface.

Ignoring rotational compensation, a simplified relation is


$$
\dot\theta_x \approx \frac{v_y}{h}
$$


$$
\dot\theta_y \approx -\frac{v_x}{h}
$$


with frame/sign factors.

The full model depends on:

- body velocity;
- attitude;
- sensor angular rate;
- sensor lever arm;
- height above ground;
- flow quality.

The supplied wrapper also highlights PX4's sign convention: the raw sensor flow sign
is reversed to match the EKF's line-of-sight rotation convention.

### 28.1 Why range/terrain matters

Flow provides an angular rate. Converting angular rate into linear velocity requires a
distance scale.

Therefore optical flow and terrain/range information are strongly coupled.

Without good height above ground, horizontal velocity inferred from flow becomes
uncertain.

---

## 29. Range Finder and Terrain State

A downward range sensor measures approximately


$$
z_r \approx h_{\text{terrain}} - p_D
$$


after correcting for attitude and sensor geometry.

In NED, both vehicle vertical position and terrain vertical position are naturally
represented in the Down direction.

The symbolic derivation uses the same basic relationship for height above ground:


$$
h_{\text{AGL}}
=
h_T - p_D
$$


depending on exact sign/state naming.

### 29.1 Terrain is not the same as vehicle altitude

The estimator can move vertically while terrain is fixed, or fly over changing
terrain while vehicle global altitude is nearly constant.

Separating the terrain state allows both effects to be represented.

### 29.2 Conditional range aiding

Range is highly accurate near the ground but can be unsuitable at large altitude,
large tilt, fog, poor quality, or outside sensor limits.

EKF2 therefore uses gating and mode logic to decide when range should constrain the
main height state and/or only update the terrain estimate.

---

## 30. True Airspeed Fusion

Airspeed relates ground-relative velocity and wind:


$$
v_{\text{air}}^n =
v^n - w^n
$$


True airspeed magnitude is approximately


$$
V_T =
\left\|
v^n -
\begin{bmatrix}
w_N\\
w_E\\
0
\end{bmatrix}
\right\|
$$


So the predicted airspeed is a nonlinear function of:

- navigation velocity;
- horizontal wind.

Airspeed fusion therefore makes wind observable.

---

## 31. Synthetic Sideslip Fusion

For many fixed-wing aircraft, coordinated flight approximately implies zero sideslip:


$$
\beta \approx 0
$$


The predicted relative airflow is


$$
v_{\text{rel}}^n = v^n - w^n
$$


rotated into body axes:


$$
v_{\text{rel}}^b
=
R(q)^T v_{\text{rel}}^n
$$


and


$$
\beta
=
\operatorname{atan2}
(v_{\text{rel},y}^b,
 v_{\text{rel},x}^b)
$$


Using a synthetic observation


$$
z_\beta = 0
$$


provides information about horizontal wind even without directly measuring sideslip.

This assumption is useful only when the aircraft actually operates close to
coordinated flight.

---

## 32. Multirotor Drag Fusion

For a multirotor, body specific force caused by aerodynamic drag contains information
about air-relative velocity.

A simplified model combines bluff-body drag and momentum drag:


$$
f_d
\sim
-\frac{1}{2}\rho C_D v_{\text{rel}}
\|v_{\text{rel}}\|
-
C_M v_{\text{rel}}
$$


in body horizontal axes.

The core symbolic derivation includes such a drag measurement model.

This allows wind estimation without a conventional airspeed sensor, provided the drag
coefficients are adequately tuned.

---

## 33. Ranging Beacon Fusion

A ranging beacon provides a scalar distance between the vehicle and a known beacon
location.

The predicted range is of the form


$$
\hat r =
\|p_{\text{vehicle}} - p_{\text{beacon}}\|
$$


after expressing both in a consistent coordinate system.

The innovation is


$$
\nu_r = \hat r - r_{\text{measured}}
$$


A single range constrains only one geometric direction. Multiple beacon geometries
and vehicle motion are needed for strong position observability.

---

## 34. Auxiliary Velocity

The wrapper can use landing-target relative motion as an auxiliary horizontal
velocity observation when the target is static and relative velocity is valid.

If the target is fixed in the world,

```text
vehicle velocity relative to target
    = - target velocity relative to vehicle
```

which explains the sign reversal in the wrapper before passing the observation to the
EKF.

---

# Initialization and Observability

## 35. Initialization

Before normal navigation, the EKF must establish a sufficiently meaningful initial
state.

Typical initialization needs include:

- valid IMU data;
- tilt alignment;
- a height source;
- a yaw/heading mechanism or conditions that make yaw observable;
- valid timing and finite sensor data.

### 35.1 Tilt from gravity

When nearly stationary,


$$
a_m \approx -g^b
$$


so roll and pitch can be initialized from the measured gravity direction.

Yaw cannot be obtained from gravity alone.

### 35.2 Yaw initialization

Yaw may come from:

- magnetometer;
- dual-antenna GNSS yaw;
- external vision;
- motion-based GNSS/IMU yaw estimation.

---

## 36. Observability: The Most Important EKF Concept

A state being present does not mean it can always be estimated.

A state is observable only when available measurements and vehicle motion cause
changes that distinguish that state from other possible errors.

Examples:

### Gyro bias

A yaw gyro bias can be distinguished from true yaw motion only when another
measurement constrains heading or horizontal motion.

### Accelerometer bias

Accelerometer bias is difficult to separate from attitude error and actual vehicle
acceleration without suitable aiding and motion.

### Magnetometer bias

A constant body magnetic bias becomes observable when the aircraft rotates relative
to the earth field.

### Wind

Wind needs air-relative information, for example:

- airspeed;
- sideslip assumption;
- drag specific force.

GNSS ground velocity alone cannot separate wind from aircraft airspeed.

### Terrain

Terrain height becomes observable when range or another ground-relative measurement
is available.

Good EKF tuning therefore requires asking:

> Which measurements constrain this state under this vehicle motion?

before asking:

> What noise parameter should I change?

---

# Fusion Control, Rejection, and Resets

## 37. Fuse, Reject, Stop, or Reset

Real navigation estimators need more than the textbook Kalman equations.

For an aiding source, EKF2 can conceptually choose among:

```text
FUSE
    measurement is valid and statistically consistent

REJECT
    measurement arrived, but innovation is outside gate

STOP FUSION
    source no longer meets continuing conditions

RESET
    normal fusion has failed for too long, but source is good enough to
    re-anchor one or more states
```

This is one of the most important practical differences between an academic EKF
example and a flight-quality estimator.

---

## 38. State Resets

A reset deliberately changes a state by a finite amount, for example:

- position reset to GNSS;
- velocity reset to GNSS;
- yaw reset to emergency yaw estimate;
- height reset to baro/GNSS/range/vision;
- terrain reset.

The wrapper publishes both:

- a reset counter;
- the most recent delta caused by the reset.

Examples include:

```text
delta_xy
delta_z
delta_vxy
delta_vz
delta_heading
delta_q_reset
delta_dist_bottom
```

Controllers can use these deltas to preserve continuity of their own internal
references.

Without reset metadata, a controller might interpret a coordinate correction as a
real physical jump.

---

## 39. Dead Reckoning

When external aiding is unavailable, the estimator can continue with inertial
propagation.

But inertial errors grow:

- attitude error grows from gyro noise/bias;
- velocity error integrates acceleration error;
- position error integrates velocity error.

The wrapper publishes dead-reckoning state through local/global position outputs.

A no-aid timeout determines when the horizontal navigation solution can no longer be
considered valid.

Dead reckoning is therefore a **degraded mode**, not an indefinitely accurate mode.

---

# EKF-GSF Emergency Yaw Estimator

## 40. Why a Separate Yaw Estimator Exists

PX4 can run an additional emergency yaw estimator based on a Gaussian Sum Filter.

The idea is to maintain several hypotheses for yaw, with small EKFs estimating
horizontal velocity and yaw.

GNSS horizontal velocity provides evidence that makes some yaw hypotheses more
consistent than others.

A weighted mixture produces a composite yaw estimate.

This gives PX4 a path to recover from a badly wrong magnetometer-based yaw estimate
without requiring the main EKF to remain locked to the faulty heading.

The wrapper publishes `yaw_estimator_status` and exposes whether an emergency yaw
estimate is available.

---

# Multi-Instance EKF Theory

## 41. Why Run Multiple EKFs?

A single EKF can reject a bad aiding observation, but failures in the IMU feeding the
prediction model are harder.

Examples:

- gyro bias step;
- accelerometer bias step;
- stuck sensor;
- saturation/clipping;
- excessive sensor noise.

PX4 can run different EKF instances with different IMU/magnetometer combinations and
compare their internal consistency.

Each instance publishes its own:

- attitude;
- local position;
- global position;
- odometry;
- status;
- innovations;
- sensor IDs.

`EKF2Selector` then chooses the primary estimator.

---

## 42. Selector Health Metric

The supplied `EKF2Selector.cpp` uses the estimator innovation test ratios.

Invalid/non-positive velocity, position, or height test ratios are replaced with
`1.0` for selector scoring.

It then computes


$$
T_{\text{combined}}
=
\max\left(
\frac{T_v + T_p}{2},
T_h
\right)
$$


where

- $T_v$ = velocity test ratio;
- $T_p$ = horizontal position test ratio;
- $T_h$ = height test ratio.

In code:

```cpp
combined_test_ratio =
    max(0.5f * (vel_test_ratio + pos_test_ratio),
        hgt_test_ratio);
```

The instance is considered basically healthy when

```text
filter_fault_flags == 0
and
combined_test_ratio > 0
```

with a hysteresis mechanism to avoid instant state flipping.

A warning is set when


$$
T_{\text{combined}} \ge 1
$$


which corresponds to innovation consistency reaching/exceeding the statistical
acceptance boundary.

---

## 43. Selector Relative Error Score

The selector compares each alternate instance with the selected primary.

For alternate instance $i$,


$$
\Delta T_i =
T_i - T_{\text{primary}}
$$


If the alternate is worse, the relative score is increased.

If the alternate is better by more than a configured margin, the score is decreased.

The score is accumulated over time/updates and constrained to


$$
-1 \le E_i \le +1
$$


in the supplied source.

This accumulation avoids switching because of a single noisy sample.

The selector requires a significant negative relative score before switching away
from an otherwise healthy primary.

---

## 44. Anti-Chatter Logic

Estimator selection must not oscillate rapidly.

The supplied source contains several mechanisms that reduce chatter:

- relative-score accumulation instead of one-sample comparison;
- minimum required improvement;
- score saturation;
- health hysteresis;
- time since an alternate was last selected;
- time since the last instance change;
- special handling when the primary has a warning;
- immediate handling of hard faults/timeouts.

This is analogous to debouncing plus hysteresis in a safety-critical state machine.

---

## 45. Estimator Timeout

The selector treats an instance as timed out if estimator status stops updating
within the configured source-code timeout window.

A stale estimator must not remain primary merely because its last published numbers
look good.

---

## 46. Multi-IMU Fault Detection

The selector also uses `sensors_status_imu` to compare physical IMUs.

### 46.1 Gyro inconsistency accumulation

For each gyro it integrates excess inconsistency above a rate threshold:


$$
E_{\omega,i}
\leftarrow
\max\left(
0,\;
E_{\omega,i}
+
(\epsilon_{\omega,i}-\epsilon_{\omega,\text{thr}})\Delta t
\right)
$$


When the accumulated angle error exceeds a configured threshold, an IMU fault is
declared.

### 46.2 Accelerometer inconsistency accumulation

Likewise,


$$
E_{a,i}
\leftarrow
\max\left(
0,\;
E_{a,i}
+
(\epsilon_{a,i}-\epsilon_{a,\text{thr}})\Delta t
\right)
$$


and it is compared with an accumulated velocity-error threshold.

### 46.3 Why at least three sensors help

With two disagreeing sensors, the system knows there is disagreement but cannot
determine which one is wrong from pairwise comparison alone.

With three or more sensors, the largest outlier relative to the group can be
identified much more reliably.

---

## 47. Preference for a Different IMU on Failover

When the primary estimator becomes unhealthy, the selector prefers a healthy
candidate using a different accelerometer/IMU when possible.

That is exactly what redundancy should do: do not fail over from one estimator to
another estimator that depends on the same failed physical sensor unless no better
option exists.

---

## 48. Preserving Output Continuity During Instance Switching

Switching from EKF instance A to instance B can create discontinuities because the
two instances may have slightly different:

- attitude;
- position;
- velocity;
- height;
- terrain estimate.

`EKF2Selector` therefore recomputes and propagates reset deltas/counters while
republishing the selected estimator.

This allows downstream controllers to see one continuous reset history even when the
underlying estimator instance changes.

The selector also rejects stale/non-monotonic data before publishing system-level
topics.

---

# Sensor Bias and Calibration Interaction

## 49. Calibration Changes

The wrapper monitors device IDs and calibration counters.

When the selected gyro/accelerometer/magnetometer calibration changes, it resets the
corresponding EKF learned bias state.

This is necessary because an EKF bias estimate learned under one calibration is not
necessarily meaningful after the driver calibration changes.

Conceptually:

```text
raw sensor
   |
   v
driver calibration
   |
   v
calibrated measurement
   |
   v
EKF residual bias estimate
```

Changing the driver calibration changes the baseline around which the EKF bias is
defined.

---

## 50. In-Flight Learned Calibration

The wrapper tracks whether sufficiently stable/valid EKF bias estimates have been
learned.

It can make them available for calibration persistence when:

- the bias is valid;
- variance is sufficiently small;
- learning conditions are valid;
- enough time has accumulated.

This connects online state estimation with long-term sensor calibration.

---

# Diagnostics

## 51. Innovations

The wrapper publishes innovations for aiding sources such as:

- GNSS velocity;
- GNSS position;
- external-vision velocity;
- external-vision position;
- baro height;
- range height;
- auxiliary velocity;
- optical flow;
- heading;
- magnetic field;
- gravity;
- drag;
- airspeed;
- sideslip.

Innovation plots are often the fastest way to identify a bad sensor, wrong delay, or
wrong frame.

---

## 52. Innovation Variances

An innovation without its variance is incomplete information.

For example:

```text
innovation = 2 m
```

could be

```text
very bad  if expected sigma = 0.2 m
reasonable if expected sigma = 5 m
```

Therefore always inspect innovation variance or test ratio together with the raw
innovation.

---

## 53. Filter Fault Flags

The wrapper publishes the core `fault_status` bitfield.

These flags indicate numerical/algorithmic faults such as invalid covariance updates
or failures in particular fusion calculations.

This is different from an observation simply being rejected by a statistical gate.

```text
observation rejected
    -> sensor disagreed with filter

filter fault
    -> internal EKF update became numerically/algorithmically invalid
```

---

## 54. Control Status Flags

Control-status flags show which modes are active, for example:

- tilt aligned;
- yaw aligned;
- GNSS position fusion;
- GNSS velocity fusion;
- barometric height fusion;
- range height fusion;
- optical flow;
- external-vision position/velocity/yaw;
- magnetometer;
- airspeed;
- wind;
- dead reckoning;
- terrain fusion;
- fake position/height;
- gravity vector fusion.

These flags are often more useful than simply checking whether a sensor topic exists.

A sensor can be present but **not currently fused**.

---

## 55. Aid-Source Status

Per-aid-source messages expose quantities such as:

- observation;
- observation variance;
- innovation;
- innovation variance;
- test ratio;
- whether it was fused;
- whether the innovation was rejected;
- sample timestamp.

For debugging, these are often the closest thing to an oscilloscope probe inside the
Kalman update.

---

## 56. Time Slip

The wrapper integrates IMU delta time and compares it against sample timestamps.

Conceptually:


$$
t_{\text{slip}}
=
(t_{\text{sample}}-t_0)
-
\sum_k \Delta t_k
$$


A growing time slip can indicate timing/dropout problems in the IMU data path.

Estimator timing faults can cause navigation errors even if the numerical sensor
values look reasonable.

---

# Parameter Theory

## 57. Noise Parameters

Noise parameters answer:

> How uncertain is this sensor/model?

Examples:

- gyro measurement noise;
- accelerometer measurement noise;
- GNSS position noise;
- GNSS velocity noise;
- barometer noise;
- range noise;
- vision position/velocity/yaw noise;
- magnetic-field noise;
- airspeed noise;
- optical-flow noise.

They directly or indirectly affect $R$, $Q$, Kalman gain, and gating behavior.

---

## 58. Process-Noise Parameters

Process noise answers:

> How quickly do I expect this state to change for reasons not explicitly modeled?

Examples:

- gyro-bias random walk;
- accelerometer-bias random walk;
- wind variation;
- terrain variation.

Too small:

- state becomes overconfident;
- slow to adapt;
- innovations grow when reality changes.

Too large:

- state becomes noisy;
- estimate follows measurements too aggressively;
- uncertainty grows quickly between observations.

---

## 59. Gate Parameters

Gate parameters answer:

> How statistically inconsistent may a measurement be before I reject it?

Larger gate:

- fewer rejections;
- more tolerance of sensor transients;
- greater chance of fusing a bad measurement.

Smaller gate:

- stronger outlier protection;
- greater chance of rejecting legitimate maneuver-related/modeling error.

Do not use gate size to hide a fundamentally wrong delay, frame, scale, or sensor
noise model.

---

## 60. Delay Parameters

Delay parameters answer:

> At what physical time did this measurement represent the vehicle state?

A delay error creates maneuver-correlated innovations.

Typical symptom:

```text
hover / straight flight: looks acceptable
aggressive maneuver: innovation spikes with motion
```

Before increasing sensor noise to hide such spikes, verify timing.

---

## 61. Sensor Position Parameters

Lever-arm parameters answer:

> Where is the sensor relative to the IMU/body reference point?

These matter most during angular motion.

A wrong lever arm may look harmless while hovering and fail during fast yaw/roll.

---

## 62. Output-Predictor Time Constants

`EKF2_TAU_POS` and `EKF2_TAU_VEL` tune how the low-latency current-time output follows
corrections produced at the delayed fusion horizon.

They do **not** replace the EKF process/measurement noise tuning.

---

# Practical Reasoning Guide

## 63. If Position Innovations Are Large

Check in this order:

1. sensor timestamp/delay;
2. coordinate frame and sign;
3. sensor lever arm;
4. sensor reported accuracy;
5. vibration / IMU clipping;
6. yaw correctness;
7. sensor multipath/outliers;
8. only then measurement-noise/gate tuning.

---

## 64. If Velocity Innovation Rises During Acceleration

Possible causes include:

- incorrect yaw;
- accelerometer scale/bias error;
- timing mismatch;
- vibration aliasing;
- GNSS velocity degradation.

A yaw error is particularly visible because inertial acceleration is rotated into the
wrong horizontal direction.

---

## 65. If Height Drifts

Investigate:

- accelerometer clipping;
- vertical vibration/aliasing;
- barometer pressure disturbances;
- wrong height reference;
- GNSS vertical quality;
- range validity;
- external-vision vertical drift;
- incorrect sensor delays.

---

## 66. If Heading Jumps

Investigate:

- magnetic interference;
- magnetometer calibration;
- GNSS yaw validity;
- external-vision yaw resets;
- emergency yaw reset events;
- EKF instance changes.

Use reset counters and information-event flags to distinguish a deliberate estimator
reset from uncontrolled divergence.

---

## 67. If EKF2Selector Keeps Switching

Inspect:

- each instance's combined test ratio;
- relative test ratio;
- filter fault flags;
- estimator timeout;
- gyro/accelerometer inconsistency accumulators;
- device IDs;
- whether the same physical IMU is shared between candidates.

Frequent switching usually indicates a real consistency problem or badly mismatched
sensor quality, not merely a selector problem.

---

# Execution Flow in the Supplied Wrapper

## 68. `EKF2::Run()` Conceptual Sequence

The supplied wrapper follows roughly this sequence:

```text
1. Handle parameter updates
   |
2. Register IMU callback if needed
   |
3. Handle estimator-related vehicle commands
   |
4. Read newest IMU sample
   |
5. Reset learned bias if calibration/device changed
   |
6. _ekf.setIMUData(...)
   |
7. Read/push aiding sensors:
      airspeed
      aux velocity
      barometer
      external vision
      optical flow
      GNSS
      magnetometer
      range finder
      ranging beacon
      system flags
   |
8. _ekf.update()
   |
9. If core update completed:
      publish local position
      publish odometry
      publish global position
      publish sensor biases
      publish wind
      publish status/events/fusion control
      publish verbose innovations/states if enabled
      update learned calibrations
   |
10. Publish attitude using current output predictor
   |
11. Publish EKF2 sensor timestamp diagnostics
```

This ordering shows the distinction between:

```text
measurement ingestion
        vs
core EKF update
        vs
output publication
```

---

# Selector Execution Flow

## 69. `EKF2Selector::Run()` Conceptual Sequence

```text
1. Update selector parameters
   |
2. Compare IMU inconsistencies
   |
3. Update health/error score of every EKF instance
   |
4. If no primary exists:
      choose first valid instance
   |
5. If primary unhealthy:
      prefer best healthy EKF using a different IMU
   |
6. Else if a significantly better healthy estimator persists:
      switch after anti-chatter timing conditions
   |
7. Else process explicit user selection request
   |
8. Publish selector status
   |
9. Republish selected estimator as:
      vehicle_attitude
      vehicle_local_position
      vehicle_global_position
      vehicle_odometry
      wind
```

---

# A Compact Mathematical Summary

## 70. Prediction

Nominal state:


$$
x_k^- = f(x_{k-1}^+, u_k)
$$


Error covariance:


$$
P_k^- =
F_k P_{k-1}^+ F_k^T
+
G_k Q_k G_k^T
$$


where $u_k$ is primarily IMU information.

---

## 71. Measurement

Predicted observation:


$$
\hat z_k = h(x_k^-)
$$


PX4-style innovation convention:


$$
\nu_k = \hat z_k - z_k
$$


Innovation covariance:


$$
S_k = H_kP_k^-H_k^T + R_k
$$


Kalman gain:


$$
K_k = P_k^-H_k^TS_k^{-1}
$$


Statistical test, conceptually:


$$
T =
\frac{\nu^2}{g^2 S}
$$


Fuse when the innovation is acceptable and source-control conditions pass.

---

## 72. Correction

Compute error-state correction:


$$
\delta x \propto K\nu
$$


with the sign handled consistently with PX4's predicted-minus-measured innovation.

Inject into nominal state:


$$
x^+ = x^- \boxplus \delta x_{\text{inject}}
$$


For attitude this is a small quaternion/rotation composition.

Joseph covariance update:


$$
P^+
=
(I-KH)P^-(I-KH)^T
+
KRK^T
$$


Then reset the local error coordinates around the corrected nominal state.

---

# What Makes PX4 EKF2 More Than a Textbook EKF

## 73. Flight-Quality Features

The core Kalman equations are only part of the system.

PX4 adds:

- error-state attitude representation;
- generated symbolic Jacobians;
- delayed sensor fusion;
- low-latency output prediction;
- sensor quality checks;
- statistical innovation gating;
- fusion start/continue/stop logic;
- state resets after prolonged fusion failure;
- bias sub-estimators;
- terrain estimation;
- wind estimation;
- emergency yaw estimation;
- dead-reckoning detection;
- sensor lever-arm correction;
- calibration-change handling;
- covariance conditioning and fault flags;
- multi-EKF redundancy and selection;
- reset propagation to downstream controllers;
- rich uORB diagnostics.

These mechanisms are what make the estimator usable on a real aircraft with delayed,
noisy, intermittent, and occasionally faulty sensors.

---

# Source-to-Theory Map for the Supplied Files

## 74. `EKF2.hpp`

Most useful for understanding:

- which sensors/features can be compiled;
- which uORB topics are consumed/published;
- EKF2 parameter groups;
- the ownership relationship between `EKF2` and `Ekf`;
- bias/calibration tracking;
- multi-instance topic behavior.

---

## 75. `EKF2.cpp`

Most useful for understanding:

- runtime sensor ingestion;
- IMU sample construction;
- measurement timestamp handling;
- sensor frame/noise preprocessing;
- publication of innovations and variances;
- reset-counter publication;
- output predictor use;
- delay validation;
- external estimator commands;
- sensor calibration-change handling.

---

## 76. `EKF2Selector.hpp`

Most useful for understanding:

- per-instance state;
- health hysteresis;
- score limits;
- supported estimator count;
- IMU inconsistency accumulation;
- system-level reset bookkeeping.

---

## 77. `EKF2Selector.cpp`

Most useful for understanding the exact selector behavior:


$$
T_{\text{combined}}
=
\max
\left(
\frac{T_v+T_p}{2},
T_h
\right)
$$


plus:

- health/fault/timeout handling;
- relative score integration;
- alternate-estimator selection;
- preference for different-IMU failover;
- reset-delta continuity;
- stale sample rejection.

---

# Recommended Reading Order for the Full EKF2 Library

If studying the implementation, a productive order is:

```text
1. EKF2.hpp / EKF2.cpp
   Understand PX4 interface and data flow.

2. EKF/estimator_interface.*
   Understand sample buffers and delayed horizon.

3. EKF/ekf.h / core update code
   Understand state machine and central EKF control.

4. EKF prediction code
   Understand nominal inertial navigation propagation.

5. generated/symbolic covariance prediction
   Understand error-state covariance propagation.

6. each aid_sources/* implementation
   Understand measurement model, innovation, gate, start/stop/reset logic.

7. output predictor
   Understand delayed-EKF to current-time control output.

8. EKFGSF_yaw
   Understand emergency yaw estimation.

9. EKF2Selector.*
   Understand redundancy/failover.
```

---

# References

## PX4 official documentation

- PX4 EKF2 overview and tuning:
  <https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf>

- PX4 estimator module reference:
  <https://docs.px4.io/main/en/modules/modules_estimator>

## PX4 source

- Core EKF class:
  <https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF/ekf.h>

- Current symbolic EKF derivation:
  <https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF/python/ekf_derivation/derivation.py>

## Error-state quaternion theory

The current PX4 symbolic derivation explicitly references:

Joan Solà, **Quaternion kinematics for the error-state Kalman filter**, 2017.

<https://arxiv.org/abs/1711.02508>

---

# Final Mental Model

The simplest correct mental model of PX4 EKF2 is:

```text
The IMU predicts how the aircraft moves.

The covariance predicts how uncertain that prediction becomes.

Every aiding sensor predicts what it should observe from the current state.

The difference between prediction and observation is the innovation.

The innovation variance says how surprising that difference is.

The gate decides whether the difference is statistically believable.

The Kalman gain decides how the accepted innovation should correct every
correlated state.

The EKF performs that fusion in the past, at a delay-compensated fusion horizon.

An output predictor carries the corrected solution forward to the present with
low latency.

Reset logic handles situations where normal statistical fusion can no longer
recover quickly enough.

Multiple EKF instances provide redundancy against bad physical sensors, and
EKF2Selector chooses the most internally consistent solution while preserving
continuous system-level reset bookkeeping.
```

That combination — inertial prediction, error-state covariance, delayed aiding,
statistical consistency checks, robust reset logic, and redundant estimator
selection — is the essential theory behind PX4 EKF2.
