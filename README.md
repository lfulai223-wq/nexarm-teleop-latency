# nexarm-teleop-latency

Measuring and reducing end-to-end latency in leader–follower arm teleoperation:
the delay between the operator moving the leader arm and the follower arm being
commanded to match.

> **Status: early.** This repository does not contain its own code yet. The
> working starting point is the reference implementation described below.

## Reference implementation

`ref-internship-teleop~/` is a local clone of
[leader-follower-teleop](https://github.com/lfulai223-wq/leader-follower-teleop),
an Alicia-D leader arm driving a FAIRINO FR3 follower over `ServoJ`, with a
Gloria-M gripper. It is a separate git repository and is **not tracked** by
this one, so a fresh clone of this repo will not include it:

```bash
git clone https://github.com/lfulai223-wq/leader-follower-teleop.git 'ref-internship-teleop~'
```

| File | Purpose |
|------|---------|
| `alicia_teleop_fr3.py` | Main teleop loop: filtering, spring tracking, prediction, latency probe, gripper |
| `gripper_teleop.py` | Standalone gripper teleop (MIT torque mode, torque-limited) |
| `record.py` | Record leader trajectories (`single`, `dataset`) and analyse them (`analyze`) |
| `integration_example.py` | Consume live leader data via `LeaderArmStream` (callback or queue) |
| `sim_fr3.py` | Mock FR3 with a live matplotlib view, used by `--sim` |
| `tremor_suppression_test.py` | Numerical check of 8–12 Hz tremor attenuation by the two filter stages |
| `Spline/` | Spline and servo experiments |

Its own README predates the latency probe, the look-ahead prediction, the
gripper channel and `record.py analyze`, and its defaults table is out of date.
The `CONFIG` block at the top of `alicia_teleop_fr3.py` is the source of truth.

## Where the latency comes from

Each stage of the reference pipeline trades smoothness for delay:

```
leader raw angles
   ├─▶ [1] OneEuro adaptive low-pass     FILTER_MINCUTOFF, FILTER_BETA
   ├─▶ [2] fixed first-order low-pass    TREMOR_CUTOFF
   ├─▶ [3] axis mapping + joint limits
   ├─▶ [4] critically damped spring      SPRING_OMEGA, MAX_ACCEL, MAX_VEL
   ├─▶ [5] look-ahead prediction         PREDICT_LOOKAHEAD_MS  (subtracts delay)
   └─▶ ServoJ @ RATE ─▶ follower
```

Rough contributions at the current defaults (theoretical, for steady motion):

| Stage | Default | Approximate delay |
|-------|---------|-------------------|
| Control period | `RATE = 125` Hz | 8 ms per cycle |
| OneEuro | `FILTER_MINCUTOFF = 3.0` Hz, `FILTER_BETA = 0.05` | up to ~53 ms at rest, less as speed rises |
| Fixed low-pass | `TREMOR_CUTOFF = 5.0` Hz | ~32 ms (1 / 2πf) |
| Spring | `SPRING_OMEGA = 20` rad/s | ~100 ms (2 / ωₙ) |
| Prediction | `PREDICT_LOOKAHEAD_MS = 10` | −10 ms |

The spring is the largest single term. Raising `SPRING_OMEGA` or
`TREMOR_CUTOFF` cuts delay at the cost of tremor rejection and smoothness.

## Measuring it

The teleop loop has a built-in probe, on by default. For each joint it times
how long it takes from the raw leader angle moving more than a threshold (2°)
until the angle sent to `ServoJ` has moved by the same amount, and prints:

```
[延迟埋点] J3 端到端延迟: 96.0 ms
```

Two things to know when reading these numbers:

- It stops at **command sent**. Follower controller and mechanical lag are not included.
- It times a displacement, so slow drags inflate the result with the time the
  hand itself took to travel 2°. Use quick, decisive single-joint moves.

Relevant flags: `--no-latency-probe`, `--latency-probe-threshold`,
`--no-predict`, `--predict-lookahead-ms`, plus `--spring-omega`,
`--tremor-cutoff`, `--filter-mincutoff`, `--filter-beta`, `--no-filter`.

## Running the reference

Requirements: Python 3.10+, `numpy`, `matplotlib` (for `--sim`), and three
vendor SDKs that are not bundled: `alicia_d_sdk` (leader), `gloria_m_sdk`
(gripper) and the FAIRINO SDK, importable as `fairino` or placed under
`Spline/fairino390/linux/`. The leader and gripper SDKs are imported at module
load, so they are needed even for dry runs.

```bash
cd 'ref-internship-teleop~'

# Leader only: print mapped targets, no follower, no gripper
python alicia_teleop_fr3.py --no-robot --no-gripper

# Mock follower with live plot
python alicia_teleop_fr3.py --sim --no-gripper

# Real hardware (teach pendant in AUTO, robot enabled, faults cleared)
python alicia_teleop_fr3.py --robot-ip 192.168.57.2 --gripper-port /dev/ttyACM1

# A/B the prediction stage
python alicia_teleop_fr3.py --sim --no-gripper --no-predict
python alicia_teleop_fr3.py --sim --no-gripper --predict-lookahead-ms 20

# Record a trajectory, then report how much of it is accelerating or reversing
python record.py single --output trajectories/demo_001.csv
python record.py analyze trajectories/demo_001.csv
```

## Safety

The follower moves in real time. Keep the workspace clear and a stop within
reach, validate axis mapping with `--no-robot` or `--sim` first, and note that
the current defaults (`MAX_VEL = 150` °/s, `MAX_ACCEL = 800` °/s²) are tuned
for low latency and are above the reference's stated standard values of 60 and
500.
