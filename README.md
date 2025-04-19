# MissionPlanner_SwarmControl

[![License: GPL-3.0](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.html)

The aim of this repository is to develop a multi-uav controller for collborative transportation of rigid bodied suspended loads. The controller is based on differential flatness and adaptive control taken from **Sreenath, Koushil & Kumar, Vijay. (2013). Dynamics, Control and Planning for Cooperative Manipulation of Payloads Suspended by Cables from Multiple Quadrotor Robots.** and **Su, Yu-Hsiang & Bhowmick, Parijat & Lanzon, Alexander. (2023). A robust adaptive formation control methodology for networked multi-UAV systems with applications to cooperative payload transportation.**

THIS PROJECT IS SUBMITTED IN PARTIAL FULFILLMENT OF THE REQUIREMENTS FOR THE DEGREE OF BACHELOR OF ENGINEERING (MECHATRONICS ENGINEERING), FACULTY OF ENGINEERING, KING MONGKUT’S UNIVERSITY OF TECHNOLOGY THONBURI, 2024

---

## How to compile

### On Windows (Recommended)

#### 1. Install software

##### Main requirements

Currently, Mission Planner needs:

Visual Studio 2022

##### IDE

### Visual Studio Community
To compile Mission Planner, we recommend using Visual Studio. You can download Visual Studio Community from the [Visual Studio Download page](https://visualstudio.microsoft.com/downloads/ "Visual Studio Download page").

Visual Studio is a comprehensive suite with built-in Git support, but it can be overwhelming due to its complexity. To streamline the installation process, you can customize your installation by selecting the relevant "Workloads" and "Individual components" based on your software development needs.

To simplify this selection process, we have provided a configuration file that specifies the components required for MissionPlanner development. Here's how you can use it:

1. Go to "More" in the Visual Studio installer.
2. Select "Import configuration."
3. Use the following file: [vs2022.vsconfig](https://raw.githubusercontent.com/ArduPilot/MissionPlanner/master/vs2022.vsconfig "vs2022.vsconfig").

By following these steps, you'll have the necessary components installed and ready for Mission Planner development.

###### VSCode
Currently VSCode with C# plugin is able to parse the code but cannot build.

---

#### 2. Get the code

If you get Visual Studio Community, you should be able to use Git from the IDE. 
Clone `https://github.com/C-PANATORN/MissionPlanner_SwarmControl.git` to get the full code.

In case you didn't install an IDE, you will need to manually install Git. Please follow instruction in https://ardupilot.org/dev/docs/where-to-get-the-code.html#downloading-the-code-using-git

Open a git bash terminal in the MissionPlanner directory and type, "git submodule update --init" to download all submodules

Alternatively, if you have installed MissionPlanner from the main branch, you can manually replace the ".csproj" and "formation.cs" files directly from this repository and compile the code. **Note: if you use this method you need to delete all cashe memory of your previous installation**

#### 3. Build

To build the code:
- Open MissionPlanner.sln with Visual Studio
- From the Build menu, select "Build MissionPlanner"

### On other systems
Building Mission Planner on other systems isn't support currently.

---

## Control Architecture Overview

At a high level, the control system performs the following tasks in a loop:
1. **Formation Tracking**: Each UAV computes its desired position relative to the leader, using a fixed geometric offset.
2. **Feedforward Prediction**: The expected acceleration of the payload is estimated using differential flatness.
3. **Feedback Correction**: Position and velocity errors are used to compute a corrective acceleration.
4. **Tension Compensation**: Internal forces from the suspended payload are balanced using a null-space formulation.
5. **Command Generation**: The total desired acceleration is mapped to attitude and thrust commands, which are sent to each UAV.
6. **Adaptive Tuning**: Control gains are updated online based on tracking errors to improve robustness.

---

### Formation Geometry and Relative Targeting

Each follower UAV is assigned a desired offset from the leader UAV in the body frame. These offsets define the intended formation pattern (e.g., line, triangle, V-shape). As the leader moves and rotates, each follower uses this offset to determine its desired position in world coordinates.

This is computed in the following block of code:

```csharp
Vector3 off = getOffsets(mav);
double hdg = -Leader.cs.yaw * (Math.PI / 180.0);
Vector3 targetUTM = new Vector3(
    leaderUTM.x + off.x * cos(hdg) - off.y * sin(hdg),
    leaderUTM.y + off.x * sin(hdg) + off.y * cos(hdg),
    leaderUTM.z + off.z
);
```

This performs a 2D rotation of the offset vector based on the leader’s heading, ensuring that the formation maintains its shape even as the leader turns.

---

### Feedforward Acceleration via Differential Flatness

Differential flatness allows the system to compute the desired trajectory and its derivatives directly. The feedforward acceleration is derived from the leader's motion and includes linear acceleration as well as centripetal and tangential components:

```csharp
Vector3 aL = new Vector3(Leader.cs.ax, Leader.cs.ay, Leader.cs.az);
float yawRate = EstimateYawRate(Leader);
float yawAccel = (yawRate - previousYaw) / dt;
Vector3 a_cent = new Vector3(-yawRate^2 * offset.x, -yawRate^2 * offset.y, 0);
Vector3 a_tan = new Vector3(-offset.y * yawAccel, offset.x * yawAccel, 0);
Vector3 a_ff = aL + a_cent + a_tan;
```

---

### Adaptive Feedback Regulation

Position and velocity errors are used to compute a correction term. This forms the second part of the control input:

```csharp
Vector3 e_pos = targetPosition - currentPosition;
Vector3 e_vel = targetVelocity - currentVelocity;
Vector3 a_fb = e_pos * Kp + e_vel * Kv;
```

This adaptive PD controller improves resilience against disturbances and modeling errors.

---

### Compensation for Payload-Induced Forces

To stabilize the payload, internal cable tensions are computed using a pseudoinverse and null-space approach. This method ensures force equilibrium without overconstraining the system:

```csharp
Vector3 compensation = payloadController.CompensateRigidBodyDynamics(Leader, Follower, offsets);
```

This solves the system:

```
Φ T = W,  T = Φ⁺ W + N Λ
```

Where Φ is the wrench matrix, W is the desired net force and torque on the payload, and NΛ is the internal force distribution.

---

### Acceleration-to-Attitude Mapping and Command Transmission

The final acceleration vector is transformed into pitch, roll, and thrust commands:

```csharp
phi = asin(u.y / g)
theta = asin(-u.x / g)
thrust = (u.z + g) / g
```

The command is transmitted to the UAVs using MAVLink in quaternion form.

---

### Online Gain Adaptation via σ-Modification

Gains are adjusted online based on squared error magnitudes:

```csharp
Kp += gammaKp * positionError^2 * dt;
Kv += gammaKv * velocityError^2 * dt;
payloadGain += gammaPayload * tensionError * dt;
```

This allows the controller to remain robust under changing conditions.

---