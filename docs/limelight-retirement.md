# limelight-part3 retirement evaluation

Reviewed September 7, 2026. Recommendation: retire `limelight-part3` as an
active branch. Keep the preserved season snapshots, and archive the unmerged
PR tips before closing the six obsolete DECODE PRs and deleting their branches.
No changes from these PRs are needed in the BIOBUZZ base as-is. PRs and their
branches remain open/present at the conclusion of this evaluation.

## Evidence from the team's local checkout

The repository inspected was:

```text
/Users/jason/Documents/Indianola Robotics Team/Code/FtcRobotController/FtcRobotController
```

It has a clean working tree on `speed-tweaks` at
`69bb375fc5dbd471df5b0197ead2c7f670408771` (February 17, 2026, "Auto updates").
All tracked TeamCode files match that commit in the team remote. This is the
latest local team-code evidence found; the checkout alone cannot establish
which APK was last deployed to the robot.

That commit is one commit newer than the team's `DecodeFinal` tag (`37b191f`).
The existing `decode-2025-2026-final` tag still correctly matches `DecodeFinal`,
but does not include these later changes:

- Added `auto/Auto_Blue_Target_9.java` and `auto/Auto_Red_Target_9.java`.
- Blue Human "Drive to triangle": 53 inches / -31 degrees changed to
  51 inches / -33 degrees; the 115-degree twist is unchanged.
- Red Human "Drive to triangle": heading changed from 29 to 33 degrees;
  the 51-inch distance and -115-degree twist are unchanged.

The full newer checkout is preserved in the fork by the annotated tag
`archive/decode/team-checkout-2026-02-17`. Existing tags remain unchanged.
For the newest discovered DECODE reference, use:

```sh
git worktree add ../FtcRobotController-DECODE-latest archive/decode/team-checkout-2026-02-17
```

## What is already preserved from limelight-part3

`limelight-part3` ends at `41b857a`. It is an ancestor of
`decode-2025-2026-final`, with zero commits outside that preserved history.
Its complete `vision/` and `odometry/` directories and `config/VisionConfig.java`
are byte-identical to the February 17 team checkout. Retaining the branch name
adds no unique vision implementation beyond the archived robot code.

Later season changes include the shared `control/FiringController.java`, updated
TeleOp/Auto integration, firing cadence and intake fixes, alliance RPM curves,
drive improvements, and autonomous routes. `limelight-part3` is therefore an
older development baseline, not the best final-robot reference.

## Open PR evaluation

| PR | Finding against team checkout `69bb375` | Recommended disposition |
| --- | --- | --- |
| [#99](https://github.com/jasoncross/FtcRobotController/pull/99) | Its START-time reset of `selectionStartMs` is not present in the team implementation. The team records `opModeStartedMs` without resetting selection timing. The PR tip also declares `getRunningLine()` twice with the same signature, a Java compile error visible in source. | Archive as an unmerged experiment; close without merging. Reconsider the timer behavior only if a new pipeline selector needs it, with a targeted test. |
| [#106](https://github.com/jasoncross/FtcRobotController/pull/106) | Uses a `getBotPoseEstimate_wpiBlue_MegaTag2` helper path and enables pose fusion. The team instead retains an LLResult-based pose selection implementation and sets both `ENABLE_POSE_FUSION` and `USE_LLRESULT_BOTPOSE_MT2` to false. Goal-tag filtering and yaw-feeding concepts exist in the team code, but the PR is not the final implementation. | Archive the experimental tip; close without merging. Do not treat these coordinates, transforms, or enabled settings as proven competition configuration. |
| [#107](https://github.com/jasoncross/FtcRobotController/pull/107) | Another MT2 blue-pose helper approach, with a separate PoseEstimate helper and pose fusion enabled. It likewise differs from the team configuration and LLResult-based implementation. | Archive alongside #106; close without merging. Use the later team snapshot as the historical reference. |
| [#110](https://github.com/jasoncross/FtcRobotController/pull/110) | Adds readiness gating through `Feed.setFeedAllowed`, pending gate releases, and older Auto/TeleOp logic. The later team code routes firing through a shared FiringController, with RPM-window, gate-open, and feeding states. Its preset changes also differ from final season tuning. | Superseded approach; archive and close without merging. |
| [#111](https://github.com/jasoncross/FtcRobotController/pull/111) | Adds older feed-motor gating and queued single-shot/aim-nudge handling in Auto/TeleOp. Both final OpMode bases instantiate the newer shared FiringController. | Superseded architecture; archive and close without merging. |
| [#112](https://github.com/jasoncross/FtcRobotController/pull/112) | Variant of the earlier feed gating/queued-shot approach with additional presets. It predates the same shared firing controller and later tuning. | Superseded architecture; archive and close without merging. |

"Superseded" describes the later architecture, not an assertion that every PR
patch was merged or that all edge-case behavior is equivalent. The historical
PR branches were inspected as source; they were not rebuilt or tested on a robot.

## Consequence for BIOBUZZ

Keep the current clean drive/vision foundation. If Limelight 3A is selected,
build its new adapter against the current SDK and new camera placement and
field data. Do not merge the old DECODE firing mechanisms, Dashboard integration,
tag IDs, or experimental pose transforms into the new season base.
