# BIOBUZZ preseason constraints

Source: the team-supplied `BIOBUZZ_Competition_Manual_V0.pdf`, titled
"BIOBUZZ Presented by RTX - Pre-Season V0", 93 pages. Reviewed September 7,
2026. Page numbers below are the manual's printed page numbers. These notes
summarize that specific preliminary edition; they are not a substitute for the
kickoff manual, later Team Updates, or official rulings. The source PDF is not
stored in this repository.

## Software and vision

| V0 reference | Constraint | Implication for this project |
| --- | --- | --- |
| R704, p. 84 | Robot Wi-Fi communication is restricted to the permitted control system. Programming laptops must disconnect during matches. Streaming control, debugging, and telemetry data must use the FTC Driver Station application; additional logging/streaming services such as FTC Dashboard and FTControl Panels are prohibited. No continuous video stream is allowed. | Keep match telemetry in the Driver Station. Do not restore Dashboard or add another network telemetry/video server. Camera image acquisition and processing onboard the robot are distinct from video streaming over Wi-Fi. |
| R702 and Table 12-9, pp. 83-84 | Limelight 3A (`LL_3A`) is an explicitly permitted programmable vision coprocessor. Limelight 3G, OpenMV Cam, and Luxonis OAK-1 are listed as prohibited examples. Other sensor coprocessors generally cannot run team-modified firmware; manufacturer binary updates are allowed. | A new Limelight 3A adapter is an option if selected. The current `AprilTagVision` implementation is for webcams and does not implement Limelight. Do not restore the old DECODE target filters or pose calibration. |
| R707-R708, p. 86 | Supported USB vision devices must use a single image sensor; UVC webcams and permitted vision coprocessors are allowed. Stereoscopic cameras are not allowed. UVC webcams may use only their UVC stream/data. | Choose a supported camera and use the SDK camera interface. Verify resolution and calibrate the selected camera on the new robot. |
| R701, p. 83 | One robot controller is the sole source of actuator control. A Control Hub, or an Android smartphone with an Expansion Hub, may be used, with at most one additional Expansion Hub. The Control Hub is the only officially supported robot controller device. | Keep actuator commands in the robot application. Treat vision results as observations consumed by that application. |
| R103, p. 67 | An INIT OpMode may position and hold mechanisms to maintain the starting configuration. Holding may last several minutes; avoid thermal failure from stalled motors. | When mechanisms are added, define intentional initialization positions and holding behavior. The current drive-only base initializes drive power to zero. |

The base already has no Dashboard dependency and no custom network video or
telemetry server. Its vision interface processes images on the robot and
reports observations through Driver Station telemetry. SDK camera previews are
diagnostic facilities, not permission to add continuous wireless video. Review
any future preview/streaming feature against the final rules before match use.

## Robot design inputs

- **R503, p. 77:** at most 8 motors and 8 servos across all mechanisms in all
  configurations used at an event. Four drive motors leave a budget of four
  additional motors; interchangeable mechanisms also count toward the total.
- **R102, p. 67:** the stationary starting configuration must fit within an
  18-inch cube, with the stated exception for preloaded scoring elements.
- **R104, p. 67:** V0 specifies no robot weight limit.
- **R105, p. 68:** the robot must remain one assembly. Expansion limits are
  still pending; do not reuse DECODE extension limits as BIOBUZZ requirements.

## Information still pending

Sections 8-11 (pp. 60-63) are placeholders for the September 12, 2026 kickoff
release. V0 does not provide the game, arena, scoring, or match-rule details
needed to set autonomous duration, endgame timing, starting poses, routes,
scoring state machines, or field AprilTag IDs and coordinates.

Keep the default tag library empty and the autonomous template free of inherited
DECODE timings and routes. The next manual/SDK review should establish:

1. Match periods and autonomous/driver-controlled transition requirements.
2. Field geometry, tag family, IDs, sizes, and poses, if supplied for the game.
3. Scoring elements, possession limits, and allowed mechanism interactions.
4. Expansion limits and starting configurations.
5. Revisions to control-system, vision, streaming, and actuator rules.

Update these notes with the new edition and exact rule references when those
details are published. The current SDK baseline is documented in
[season-transition.md](season-transition.md).
