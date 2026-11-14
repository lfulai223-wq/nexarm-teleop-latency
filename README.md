# nexarm-teleop-latency

Measuring and reducing latency in leader–follower teleoperation of a NexArm
6-DOF bus-servo arm, with the PC in the loop: read the leader, process, write
the follower, all timestamped on one clock.

**Status:** work in progress. The driver, measurement tools and a first
compensation result are done. See [what is not done yet](#limitations-and-open-items).

## Result so far

Without compensation the follower trails the commanded trajectory by about
84 ms on average. Linear feedforward prediction brings that to about 20 ms.

| Configuration | Mean lag | P95 lag | Tracking error (RMS) | Jitter (RMS) |
|---|---|---|---|---|
| No prediction | 84.2 ms | 116.0 ms | 27.8 counts | 1.58 counts |
| Uniform 80 ms lookahead | 21.9 ms | 56.0 ms | 12.8 counts | 1.87 counts |
| Per-joint lookahead | 19.4 ms | 50.0 ms | 12.1 counts | 1.87 counts |

Joints 1–5 pooled, 366 two-second windows per row, three replays of the same
40 s recorded trajectory. One count is 1/4096 of a revolution. Conditions and
caveats are below; they matter.

**Checked against high-speed video.** The figures above come from positions
read back over the serial link being measured, and from replays that skip the
real leader. Filming live teleoperation at 240 fps (base-joint flicks of about
25°) gives the true end-to-end delay, from the leader physically moving to the
follower physically moving, at the 50% point of each move:

| Configuration | Video (ground truth) | Serial log, same runs |
|---|---|---|
| No prediction | 100.1 ms ±11.2 (12 moves) | 86.4 ms ±1.7 |
| Per-joint lookahead | 68.6 ms ±12.3 (9 moves) | 51.9 ms ±3.8 |

![End-to-end delay split into the part the serial log cannot see and the part it can](docs/img/latency_breakdown.png)

The serial log reads 14–17 ms low, about the time it takes to read the leader,
which it cannot see. Prediction saves about 31 ms end to end; the price is
follower overshoot rising from about 12% to about 32% of the move.

![Lag, tracking error and jitter against prediction lookahead](docs/img/lookahead_sweep.png)

Lag falls roughly linearly with lookahead. Tracking error bottoms out near
80 ms and rises again at 100 ms, where the prediction overshoots; jitter rises
throughout. That puts the useful range at 60–80 ms. (This sweep is from a
different, 27.5 s trajectory: six settings, three replays each.)

![Mean lag by joint for three configurations](docs/img/per_joint_lag.png)

Joints differ: the shoulder is the slowest at 109 ms, the base and wrist pitch
the fastest at 68 ms. A single lookahead leaves a 37 ms spread between joints;
per-joint values cut it to 10 ms.

## Where the delay is

| Segment | Measured |
|---|---|
| Read leader (6 joints, one request) | 13.8 ms mean, 14.1 ms P95 |
| PC processing (leader reply to command written) | about 0.08 ms |
| Read follower (6 joints, bridge mode) | 4.6 ms mean, 4.8 ms P95 |
| Follower execution, 10° step: first motion / 50% / 90% | 31–48 ms / 72–114 ms / 113–174 ms |

The servo's own response dominates. That is why filtering alone cannot help
here (it only adds delay) and prediction can.

![Follower step response for three joints](docs/img/step_response.png)

The leader read is slow because its firmware polls the six servos one at a
time with a fixed wait after each. With one leader read and one follower read
per cycle the loop tops out near 54 Hz; the experiments run at 50 Hz.

## Other things tried

Same 40 s trajectory, replayed; joints 1–5 pooled; "no prediction at 50 Hz"
is the reference row.

| Change | Mean lag | P95 lag | Tracking error | Note |
|---|---|---|---|---|
| None (50 Hz, no prediction) | 82.7 ms | 110.0 ms | 27.7 | fresh repeat of the reference, 84.2 ms a day earlier |
| Control rate 25 Hz | 95.0 ms | 119.9 ms | 32.4 | |
| Control rate 100 Hz | 76.0 ms | 115.0 ms | 24.5 | |
| Input filters (One Euro 3 Hz + 5 Hz low-pass) | 139.9 ms | 174.8 ms | 43.9 | jitter 1.58 → 1.24 |
| Input filters + per-joint lookahead | 71.7 ms | 107.4 ms | 24.3 | |
| Per-joint lookahead from a fitted search | 18.1 ms | 48.0 ms | 12.9 | wrist roll over-compensated (0.4 ms) |

- **Rate.** Doubling from 50 to 100 Hz saves about 7 ms, roughly half of the
  10 ms shorter period, and does not change smoothness (acceleration RMS on a
  common 25 Hz grid: 2603 vs 2871 counts/s²). Live teleoperation cannot use it:
  reading the leader takes 14 ms, which caps the loop near 70 Hz, or 54 Hz when
  the follower is also read back.
- **Filters.** The filters carried over from the earlier system add about
  56 ms and buy a 20% jitter reduction. The leader's encoder signal is already
  clean, so they are not worth it here at these settings.
- **Searching the lookahead.** Fitting each joint's optimum from the sweep did
  not beat the hand-set values; the fit extrapolated past the measured range
  for the two slowest joints.

## How it works

```
leader arm ──USB──▶ PC ──USB──▶ follower arm
 (ESP32)            │            (ESP32 + AT32 co-processor)
                    ├─ read 6 joint positions          t_request, t_reply
                    ├─ map leader → follower
                    ├─ predict: x + v · lookahead      (optional)
                    ├─ clamp step per cycle
                    ├─ write 6 joint positions         t_write
                    └─ read follower positions back    t_request, t_reply
```

Both arms speak a framed serial protocol at 1 Mbps. The follower is switched
into a bridge mode in which its ESP32 forwards position reads and writes to
the servo bus. Every send and receive is stamped with `time.perf_counter_ns()`.

**Measuring lag.** A hand on the leader is not repeatable, so a recorded leader
trajectory is replayed as a virtual leader. Lag is the time shift that best
aligns the commanded trajectory with the follower's measured position, fitted
per two-second window to get a distribution. Negative lags are allowed, so
over-prediction shows up instead of being clipped at zero.

**Prediction.** Velocity is a low-passed finite difference (5 Hz cutoff), and
the command is the position extrapolated by the lookahead, with a cap on the
lead.

## Repository layout

| Path | Contents |
|---|---|
| [nexarm_teleop/protocol.py](nexarm_teleop/protocol.py) | Frame build and parse, leader-to-follower mapping |
| [nexarm_teleop/driver.py](nexarm_teleop/driver.py) | Serial driver with timestamps on every transfer |
| [nexarm_teleop/predictor.py](nexarm_teleop/predictor.py) | Linear feedforward prediction, per-joint lookahead |
| [nexarm_teleop/latency.py](nexarm_teleop/latency.py) | Time-shift lag estimation, stale-reading detection |
| [scripts/](scripts/) | Hardware checks, teleoperation, step response, replay, analysis, video check, figures |
| [tests/](tests/) | Unit tests that need no hardware |
| [results/](results/) | Summary tables behind the numbers above |
| [docs/roadmap.md](docs/roadmap.md) | Plan and the full measurement log, including dead ends (Chinese) |

## Running it

Needs Python 3.12, a NexArm leader and follower on USB, and membership of the
`dialout` group.

```bash
python3 -m venv .venv
.venv/bin/pip install numpy matplotlib pyserial pytest

.venv/bin/python -m pytest tests                 # no hardware needed

.venv/bin/python scripts/probe_ports.py          # read-only: which port is which
.venv/bin/python scripts/check_bridge.py         # read-only: read timing on both arms
.venv/bin/python scripts/teleop.py --duration 40 # teleoperate and record a trajectory
.venv/bin/python scripts/step_response.py        # follower step response
.venv/bin/python scripts/replay.py results/logs/teleop_XXXX.csv --lookahead 80
.venv/bin/python scripts/replay.py results/logs/teleop_XXXX.csv --lookahead 65 110 80 65 95 0
.venv/bin/python scripts/analyze_replay.py results/logs/batch2/replay_la*.csv
.venv/bin/python scripts/video_latency.py media/flick_none.MOV --log results/logs/teleop_XXXX.csv
.venv/bin/python scripts/make_figures.py
```

The video check needs `opencv-python-headless` as well.

Port paths are set in [nexarm_teleop/\_\_init\_\_.py](nexarm_teleop/__init__.py)
and are specific to one machine: the two boards have identical USB-serial
chips with no serial number, so they are told apart by USB port.

The follower must not be in its wireless (ESP-NOW) sync mode while the PC is
driving it, or the two command sources fight.

**Safety.** The follower moves under program control. Keep its workspace
clear. The scripts refuse to start if leader and follower poses differ too
much, ramp in over a few seconds, limit the step per cycle, and the step test
aborts if a joint stops short of its target.

## Test conditions

- Control rate 50 Hz (achieved 50.0 Hz in every replay, no timeouts).
- Servo motion parameters: acceleration 100, speed limit 2000 (the vendor
  defaults). On a 10° step these made no measurable difference.
- No payload. Arm clear of the table.
- Trajectories are hand-dragged leader motion, replayed: peak joint speed about
  130–200 °/s.
- Lag is fitted on two-second windows with a one-second hop, keeping windows
  with at least 80 counts of motion.
- Timestamps for a position are the midpoint of its request and reply.

## Limitations and open items

- **Replay lag excludes the leader read.** Replay bypasses the real leader;
  the video check puts the missing part at 14–17 ms.
- **The video check is coarse.** One joint, about ten moves per configuration,
  no markers on the arms. Per-move scatter is ±11–12 ms and the two flick
  directions differ by about 18 ms in the video but only 2 ms in the log, so
  the tracking has a direction-dependent error. Treat the means, not single
  moves, as the result.
- **Moderate motion only.** No fast moves or hard stops, and overshoot is not
  measured separately.
- **The gripper is not compensated.** Its lag is about 140 ms and is set by
  rate limiting, not by anything prediction can fix: the leader gripper maps
  to the follower with a gain of 4.
- **The follower sometimes returns a stale reading** for joint 2 (the same
  value for several cycles, then a jump faster than the servo's speed limit).
  These samples are detected and dropped from the analysis.
- **Per-joint lookahead values were set by hand** from one measurement. A
  fitted search was tried and did no better; a proper search would sweep each
  joint past 100 ms.
- Not done: the compatibility write-up for the stock LeRobot SO-101 driver,
  and lighter filter settings that might trade less delay for the jitter
  reduction.

## Background

This applies a measure-then-compensate method used earlier on a different
pair of arms (an Alicia-D leader driving a FAIRINO FR3 over `ServoJ`,
[leader-follower-teleop](https://github.com/lfulai223-wq/leader-follower-teleop)).
The hardware, the protocol and where the delay sits are all different here,
so nothing was carried over as a number; everything above was measured on
NexArm.
