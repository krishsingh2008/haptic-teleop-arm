# Haptic Teleop Arm

A robot arm that mirrors my arm and hand in real time, with a glove that pushes back on my fingers so I can feel what the robot grips.

**Status:** In progress. Phase 1 (glove controls the gripper).

This builds on [Gripatronic](https://github.com/krishsingh2008/gripatronic-glove), my high school glove that pulled the wearer's fingers closed. This version reverses that: the glove senses each finger and resists it in proportion to the robot's grip force.

## How it works

```
 Glove (leader ESP32)                       Arm (follower ESP32)
 finger pots + 3 IMUs  ── joint targets ──►  6 servos
 feedback servos       ◄── grip force ────   gripper current sensing
                         (ESP-NOW, ~100 Hz)
```

- **Arm tracking:** IMUs on my upper arm, forearm and hand give relative joint angles, which map to the robot's base, shoulder, elbow and wrist.
- **Finger tracking:** each fingertip tendon winds on a spring-loaded spool with a potentiometer.
- **Force feedback:** grip force is estimated from gripper servo current. Micro servos on the glove spools resist finger motion in proportion to it.
- **Safety:** gripper current is capped in firmware, and the glove can only resist motion, never force fingers open.

## Roadmap

- [ ] **Phase 1:** glove v2 drives the gripper wirelessly
- [ ] **Phase 2:** 5-DOF printed arm mirrors arm motion
- [ ] **Phase 3:** force feedback to the fingers
- [ ] **Phase 4:** live dashboard, performance measurements, demo video

## Targets

| Metric | Target | Measured |
|---|---|---|
| Round-trip latency | < 50 ms | — |
| Joint tracking error | < 5° | — |
| Control loop rate | ≥ 100 Hz | — |

## Repo layout

```
firmware/      leader (glove) and follower (arm) ESP32 code
dashboard/     Python live plotting
cad/           CAD files and renders
electronics/   wiring and schematics
docs/          build log
```
