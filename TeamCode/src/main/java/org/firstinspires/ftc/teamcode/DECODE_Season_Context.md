# FTC 2025–2026 DECODE Season Context

This document provides background context for the **FTC 2025–2026 DECODE™** season, allowing developers to understand *why* the robot code is designed the way it is. It summarizes the rules, match objectives, and strategy foundations for the Indianola Robotics robot software.

---

## Overview

In **FIRST Tech Challenge (FTC) 2025–2026 DECODE™ presented by RTX**, two **alliances**—each made up of **two teams**—compete to score points by collecting and manipulating lightweight plastic whiffle balls called **ARTIFACTS**. These ARTIFACTS come in **two colors: purple and green**, and are *neutral* (not alliance-specific).

The DECODE theme emphasizes vision-based recognition, pattern decoding, and coordinated alliance play. Robots must use sensors, cameras, and programming to identify **MOTIFS** (patterns) and execute strategies that reflect the displayed sequence.

---

## Field Layout and Key Elements

Each alliance’s field half includes:

- **GOAL** – A triangular-top container where robots score ARTIFACTS.  
- **CLASSIFIER** – Attached to the GOAL; includes a **SQUARE**, **RAMP**, and **GATE** for sorting ARTIFACTS.  
- **OBELISK** – Outside the field, showing an **AprilTag** indicating the current **MOTIF** (pattern).  
- **LOADING ZONE** – Where human players load ARTIFACTS onto the field.  
- **BASE** – Zone where robots must return before match end.  
- **SECRET TUNNEL ZONE** – Area for overflow ARTIFACTS exiting the opposing ramp.

The field is a **12 ft × 12 ft (144 in × 144 in)** arena with mirrored red and blue halves.  To simplify description of the field, we use coordinates of north, south, east and west with the goals considered to be in the north east and north west corners.

---

## Match Phases and Goals

Each match lasts **2 minutes 30 seconds** and is divided into three phases:

### 1. Autonomous (30 seconds)
Robots operate solely on pre-programmed code and sensors.  
**Objectives:**
- Leave the launch line for movement points.  
- Detect the OBELISK AprilTag (IDs 21–23) to determine the **MOTIF**.  
- Score ARTIFACTS into the GOAL.  
- Arrange ARTIFACTS on the CLASSIFIER ramp matching the MOTIF pattern.  

### 2. TeleOp (2 minutes)
Drivers take manual control.  
**Objectives:**
- Collect ARTIFACTS from the loading zone.  
- Continue scoring in the GOAL.  
- Maintain or build the MOTIF pattern.  
- Prepare to return to BASE for endgame.  

### 3. Endgame (Final 20 seconds)
Triggered by a train whistle sound.  
**Objectives:**
- Return to BASE (fully or partially supported for points).  
- Bonus if both alliance robots are fully supported.  
- All motion must stop at the buzzer.

---

## Scoring Summary

| Action | Auto | TeleOp | Notes |
|--------|-------|---------|-------|
| Leave Launch Line | 3 pts | — | Move off starting tile |
| ARTIFACT – Classified | 3 pts | 3 pts | Scored correctly into GOAL |
| ARTIFACT – Overflow | 1 pt | 1 pt | Missed or excess artifact |
| ARTIFACT – Depot | — | 1 pt | Deposited but not scored |
| PATTERN (matches MOTIF) | 2 pts | 2 pts | For each correct placement |
| Return to BASE (partial) | — | 5 pts | Partially supported in BASE |
| Return to BASE (full) | — | 10 pts | Fully supported in BASE |
| Both robots fully returned | — | +10 bonus | Endgame teamwork bonus |

**Ranking Points (RP):**
- *Movement RP* – Earned for achieving combined leave/base thresholds.  
- *Goal RP* – Reached when a minimum number of ARTIFACTS are scored.  
- *Pattern RP* – Earned when PATTERN points exceed a threshold.

---

## MOTIF System and AprilTags

Before each match, the **OBELISK** displays one of three random **MOTIFS**:
- **GPP**
- **PGP**
- **PPG**

These define the correct color sequence (Green/Purple) for ARTIFACTS on the CLASSIFIER ramp.

**AprilTag IDs:**
- 20 – Blue Alliance Goal  
- 24 – Red Alliance Goal  
- 21–23 – OBELISK Motifs (GPP, PGP, PPG)

**Coding Implications:**
- Vision system should detect AprilTags 20–24.  
- OBELISK tags (21–23) identify the current pattern.  
- Goal tags (20, 24) enable auto-aim and distance calculation.  
- OBELISK tags should *not* be used for localization (position may vary).

---

## Strategic Design and Coding Goals

The DECODE challenge focuses on precision, vision processing, and coordinated alliance gameplay. Code should therefore:

- **Use vision (AprilTags)** for alignment and motif decoding.  
- **Control flywheel velocity** precisely for consistent launching.  
- **Use IMU feedback** for field-oriented drive control (mecanum).  
- **Integrate autonomous + TeleOp logic** for consistent subsystem behavior.  
- **Implement stopAll safety** so all motion ceases when the match ends.  

---

## Summary

FTC DECODE emphasizes **intelligent vision processing**, **pattern matching**, and **autonomous adaptability**.  
Teams must program robots that can *see*, *analyze*, and *act* quickly to classify ARTIFACTS, align to goals, and collaborate within strict match timing.

This document ensures that developers understand the *strategic intent* behind each robot subsystem—why IMU correction, flywheel velocity control, and vision logic are critical—and how they contribute to successful match outcomes.
