# Aruku - Biped Walking Controller

## Project Overview

Aruku is a ROS2-based biped walking controller for the Hiro robot. It uses a kinematic-based walking engine with IMU-based balance compensation and an actual_walk_phase feedback system.

---

## Walk Balancing System

### High-Level Architecture

```
RobotWrapper (taisei)
  └── Computes actual_walk_phase from foot positions via kinematic tree
  └── Publishes to /walking/walk_phase

WalkingNode (aruku)
  └── Subscribes to /walking/walk_phase, IMU data, joint states
  └── Timer at 8ms (125 Hz) drives WalkingManager

WalkingManager
  └── Runs kinematic walk engine (Kinematic class)
  └── Applies PID pitch/roll balance compensation
  └── Tracks actual_walk_phase from kinematic tree

Kinematic (process)
  └── Generates joint angles from walk trajectories
  └── Pause mechanism: freezes m_time when roll exceeds threshold
```

---

### actual_walk_phase System

**Purpose:** Compensates for lack of pressure sensors by inferring which foot is on the ground from kinematic tree data.

**Publication flow:**
```
RobotWrapper::get_walk_phase() [taisei/robot_wrapper.cpp:252-270]
  -> /walking/walk_phase topic
  -> WalkingNode subscribes [walking_node.cpp:77-80]
  -> WalkingManager::update_actual_walk_phase() [walking_manager.cpp:285-288]
  -> Kinematic::set_actual_walk_phase() [kinematic.cpp:132-134]
```

**Detection logic [robot_wrapper.cpp:252-270]:**
- Gets left/right foot positions relative to `base_footprint_world`
- Checks if foot Z-height is below threshold (`FOOT_HEIGHT_EPS = 1e-6`)
- Left contact + right contact -> DOUBLE_SUPPORT (0)
- Left contact only -> LEFT_SUPPORT (2)
- Right contact only -> RIGHT_SUPPORT (1)

**WalkPhase enum [aruku_interfaces/msg/WalkPhase.msg]:**
- `DOUBLE_SUPPORT = 0`
- `RIGHT_SUPPORT = 1`
- `LEFT_SUPPORT = 2`

**Usage in aruku:**
- Kinematic stores `actual_walk_phase` to determine swing foot timing
- WalkingManager uses it to apply ankle roll PID to the correct foot
- Kinematic's pause mechanism checks `actual_walk_phase != WalkPhase::DOUBLE_SUPPORT` before pausing

---

### PID Roll Balance (Lateral Push Compensation)

**Location:** `walking_manager.cpp:341-439`

**Data flow:**
```
IMU subscriber [walking_node.cpp:59-66]
  -> WalkingManager::update_imu(roll, pitch) [walking_manager.cpp:278-283]
  -> kinematic.update_imu_roll(roll) [kinematic.cpp:136-138]
  -> PID computed in WalkingManager::process() [walking_manager.cpp:341-371]
  -> Offset applied to ankle roll joint per support phase [walking_manager.cpp:400-416]
```

**PID computation [walking_manager.cpp:344-371]:**
```
roll_error = (|imu_roll| < roll_deadband) ? 0 : imu_roll  // deadband filter
roll_integral = clamp(roll_integral + roll_error * dt, -50, 50)
roll_integral *= 0.8  // integral decay
roll_derivative = (roll_error - prev_roll_error) / dt  // NaN guard on dt=0

pid_offset_roll = p * roll_error + i * roll_integral + d * roll_derivative
pid_offset_roll = clamp(pid_offset_roll, -180, 180)
if (y_move_amp != 0) pid_offset_roll = clamp(pid_offset_roll, -30, 30)
```

**Application to joints [walking_manager.cpp:400-416]:**
- During LEFT_SUPPORT: add `(1 - hip_ankle_ratio_roll) * pid_offset_roll` to LEFT_ANKLE_ROLL
- During RIGHT_SUPPORT: add `(1 - hip_ankle_ratio_roll) * pid_offset_roll` to RIGHT_ANKLE_ROLL

**Configuration params (from walking.json):**
- `p_roll_gain`, `i_roll_gain`, `d_roll_gain`
- `hip_ankle_ratio_roll` (split between hip and ankle compensation)
- `roll_deadband` (ignore small roll errors)

---

### Walk Pause Mechanism (Lateral Push Response)

**Location:** `kinematic.cpp:742-830` (inside `run_kinematic()`)

**Purpose:** When the robot is pushed laterally, freeze the walk cycle to let it recover balance instead of continuing into a dangerous position.

**State variables (static local):**
- `is_paused` - pause active flag
- `pause_counter` - frames spent paused
- `phase_on_pause` - which phase was active when paused (LEFT_SUPPORT or RIGHT_SUPPORT)
- `roll_has_recovered` - roll angle has come back below resume threshold

**Pause trigger [kinematic.cpp:757-762]:**
```
if (!is_paused && pause_enable && |imu_roll| > roll_pause_threshold
    && actual_walk_phase != DOUBLE_SUPPORT) {
  is_paused = true;
  phase_on_pause = actual_walk_phase;
  pause_counter = 0;
  roll_has_recovered = false;
}
```

**Resume conditions [kinematic.cpp:764-784]:**
1. Roll must recover below `roll_resume_threshold` first (`roll_has_recovered = true`)
2. Walk phase must change away from `phase_on_pause` (safe landing on other foot)
3. OR `pause_counter` exceeds `max_pause_counter` (force resume after timeout)

**Resume phase snap:** On resume, `m_time` jumps to the opposite leg's single support start to continue the walk cycle properly.

**Landing height reduction [kinematic.cpp:797-801]:**
- When paused, foot landing height is gradually reduced over 10 frames
- `landing_factor = 1.0 - (min(pause_counter, 10) / 10.0)`
- Prevents large swing foot impacts while balancing

**Position equalization during pause [kinematic.cpp:800-815]:**
- `pause_x_equalize` - 0.0 = off, 1.0 = fully equalize both legs' x to their midpoint
- `pause_y_equalize` - 0.0 = off, 1.0 = fully equalize both legs' y to their midpoint
- Makes feet symmetric during pause for stable stopping. Default 1.0.

**Reset on new cycle [kinematic.cpp:748-753]:**
- When `m_time == 0` (start of new walk period), all pause state resets

---

### m_time / Walk Phase Movement

**Concept:** `m_time` is a normalized time counter (0 to `m_period_time`) that drives the walk cycle. It increments by `time_unit` each frame and wraps at `m_period_time`.

**Phase boundaries (relative to m_time = 0 as double support start):**
- `m_ssp_time_start_l` - left single support starts
- `m_ssp_time_start_r` - right single support starts

**m_time reset on pause resume:**
- On resume, `m_time` snaps to the opposite leg's single support start (NOT from where it paused)
- This ensures the walk cycle continues with the correct phase

---

## Key Files

| File | Role |
|------|------|
| `src/taisei/robot_wrapper/robot_wrapper.cpp:252-270` | Computes actual_walk_phase from foot contact detection |
| `src/taisei/node/taisei_node.cpp:54,69-72` | Publishes walk phase to ROS topic |
| `src/aruku/walking/node/walking_node.cpp:77-80` | Subscribes to walk phase and IMU |
| `src/aruku/walking/node/walking_manager.cpp:278-370` | PID roll/pitch balance, phase tracking |
| `src/aruku/walking/process/kinematic.cpp:742-830` | Walk pause mechanism and landing height reduction |
| `src/aruku/include/aruku/walking/process/kinematic.hpp:126-138` | Balance pause config params |
| `configuration/hiro/walking/kinematic.json` | Balance pause parameters |
| `configuration/hiro/walking/walking.json` | PID gain configuration |

---

## Key Configuration Parameters

### Balance Pause (kinematic.json)
- `enable`: boolean - turn pause mechanism on/off
- `roll_pause_threshold`: degrees - trigger pause when roll exceeds this
- `roll_resume_threshold`: degrees - allow resume when roll recovers below this
- `max_pause_counter`: frames - force resume after this many frames paused
- `pause_x_equalize`: 0.0-1.0 - equalize x positions during pause (default 1.0)
- `pause_y_equalize`: 0.0-1.0 - equalize y positions during pause (default 1.0)

### PID Roll (walking.json)
- `p_roll_gain`, `i_roll_gain`, `d_roll_gain`: PID coefficients
- `hip_ankle_ratio_roll`: split between hip and ankle (0=all ankle, 1=all hip)
- `roll_deadband`: degrees - ignore roll errors below this