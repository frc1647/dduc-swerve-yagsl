# Shooting While Moving — Research & Implementation Plan

## 1. Problem Statement

We want to shoot Fuel (balls) into the Hub (a basketball-hoop-like target at 72 inches / 1.83 m off the ground) **while the robot is moving**. Our turret can rotate to aim, and we can control the flywheel speed to vary the shot velocity. The challenge is that the robot's motion imparts additional velocity to the ball at launch, so we must compensate for that motion in real time.

---

## 2. Core Concept — The "Virtual Target" Method

The most successful and widely-adopted approach (used by FRC 1706 at 2022 Worlds, among others) is the **Virtual Target** or **Moving Goal** method:

> Instead of treating the robot as moving and the goal as stationary, treat the robot as stationary and the goal as moving. Then calculate where the goal **will be** after the ball's flight time and aim there.

### How It Works

1. **Get robot pose** from odometry (fused with Limelight AprilTag vision).
2. **Get robot velocity** (`ChassisSpeeds` from the swerve drive, field-relative).
3. **Calculate distance to the real hub** and look up the **flight time** for that distance from an interpolation table.
4. **Compute the "virtual goal" position** by offsetting the real goal opposite to the robot's velocity:
   ```
   virtualGoalX = hubX - flightTime * (robotVelX + robotAccelX * kAccelCompFactor)
   virtualGoalY = hubY - flightTime * (robotVelY + robotAccelY * kAccelCompFactor)
   ```
5. **Aim the turret** at the virtual goal instead of the real goal.
6. **Look up flywheel RPM and hood angle** (if applicable) using the distance to the **virtual goal**, not the real goal.

### Why This Works

The robot's velocity is added to the ball at launch. By aiming at a point offset from the real goal, we pre-compensate so that the ball's total velocity vector (shooter + robot motion) points at the actual hub.

### The Acceleration Compensation Factor

Team 1706 found that using only current velocity caused the turret to **lag** during acceleration and deceleration. They added an acceleration term:

```
effectiveVel = currentVel + accel * kAccelCompFactor
```

- `kAccelCompFactor` ≈ 0.1 seconds (experimentally tuned)
- This predicts where the robot will be ~100ms in the future
- Accounts for latency in vision processing, CAN bus, and ball travel from feeder to shooter
- **Critical insight**: After adding this, 1706 ran in shoot-while-move mode 100% of the time at Houston Worlds

### Iterative Refinement (Optional, Advanced)

The virtual goal is at a different distance than the real goal, which means the flight time changes. For higher accuracy, you can **iterate**:

```java
for (int i = 0; i < 3; i++) {
    double newDist = robotPose.getTranslation().getDistance(virtualGoal);
    double newFlightTime = flightTimeTable.get(newDist);
    virtualGoal = recalculate(hubPos, robotVel, robotAccel, newFlightTime);
}
```

Team 1706 found this converges in 2-3 iterations and makes a noticeable improvement for shots where the robot is moving toward/away from the goal (where distance changes most).

---

## 3. Alternative Approach — Velocity Cancellation

Another approach (described by jacob6838 on CD) is to **cancel the robot's velocity** by adjusting the shot parameters:

1. Compute the **static shot** parameters (velocity, vertical angle, horizontal angle) for the current distance as if the robot were stationary.
2. Convert everything to Cartesian velocity components.
3. Solve for new shot parameters such that:
   ```
   shotVelX - robotVelX = staticVelX
   shotVelY - robotVelY = staticVelY
   shotVelZ             = staticVelZ   (robot has no vertical velocity)
   ```
4. Convert back to (speed, vertical angle, horizontal angle).

**Solution** (from jacob6838's derivation):
```
theta_h = atan2(V*sin(phi_h) + v_y, V*cos(phi_h)*cos(phi_v) + v_x)
theta_v = atan2(V*sin(phi_v), sqrt((V*cos(phi_v)*cos(phi_h) + v_x)^2 + (V*sin(phi_h) + v_y)^2))
v_s     = V*sin(phi_v) / sin(theta_v)
```

Where:
- `V, phi_v, phi_h` = static shot velocity, vertical angle, horizontal angle
- `v_x, v_y` = robot velocity components
- `v_s, theta_v, theta_h` = adjusted shot velocity, vertical angle, horizontal angle

**Tradeoff**: This is more mathematically "pure" but requires knowing the actual ball exit velocity (not just flywheel RPM), which is harder to calibrate. The virtual target method is more practical because it works with empirical lookup tables.

---

## 4. Alternative Approach — Moving Target Interception (Quadratic)

Team Spartronics (4915) used a **projectile interception** formulation:

1. Treat the hub as a moving target (with velocity = negative of robot velocity).
2. Solve the quadratic equation for intercept time `t`:
   ```
   a = Vhub_x^2 + Vhub_y^2 - shooterSpeed^2
   b = 2 * (hubPos_x * Vhub_x + hubPos_y * Vhub_y)
   c = hubPos_x^2 + hubPos_y^2
   
   t = (-b ± sqrt(b^2 - 4ac)) / (2a)
   ```
3. Pick the smallest positive `t`.
4. The aim point is `hubPos + Vhub * t`.

This assumes a **constant projectile speed** (horizontal component), which is a reasonable simplification. It's elegant but less forgiving of real-world inconsistencies than the lookup-table approach.

---

## 5. Projectile Physics — What We Need to Know

### Basic 2D Projectile Motion (No Drag)

```
x(t) = x0 + v * cos(theta) * t
z(t) = z0 + v * sin(theta) * t - 0.5 * g * t^2
```

Where:
- `v` = ball exit velocity (m/s)
- `theta` = launch angle from horizontal
- `g` = 9.81 m/s²
- `z` = vertical axis, `x` = horizontal distance

### Key Parameters to Measure/Calibrate

| Parameter | How to Determine |
|---|---|
| **Ball exit velocity vs. flywheel RPM** | Measure with a chronograph or video analysis. There is significant slip — wheel surface speed ≠ ball speed. Use the [ReCalc flywheel tool](https://www.reca.lc/flywheel) to estimate. |
| **Flight time vs. distance** | Measure empirically or calculate from projectile equations. Build an `InterpolatingTreeMap<Double, Double>` (WPILib built-in). |
| **Flywheel RPM vs. distance** | Empirical testing at known distances. Build an interpolation table. |
| **Launch angle** | Fixed by hood geometry, or adjustable if you have a hood actuator. |

### Air Drag and Magnus Effect

- Simple projectile motion (no drag) is a reasonable starting point for distances under ~15 feet.
- For longer shots, drag becomes significant. Team 5818 used a quadratic fudge factor: `hoodAngle += (dist^2) * 0.2` for far shots.
- Magnus effect (ball spin causing lift/curve) matters for single-wheel shooters with high backspin. Dual-wheel shooters with controlled compression have less spin.
- **Recommendation**: Start with no-drag physics + empirical lookup tables. Add drag compensation as a tuning offset if needed.

---

## 6. Lookup Tables We Need to Build

All tables are populated by **empirical testing** at known distances on the field.

### 6.1 Flywheel RPM Table
```java
// distance (meters) → flywheel RPM
InterpolatingTreeMap<Double, Double> rpmTable = new InterpolatingTreeMap<>();
rpmTable.put(2.0, 2800.0);
rpmTable.put(3.0, 3200.0);
rpmTable.put(4.0, 3600.0);
// ... etc, from testing
```

### 6.2 Flight Time Table
```java
// distance (meters) → time in air (seconds)
InterpolatingTreeMap<Double, Double> flightTimeTable = new InterpolatingTreeMap<>();
flightTimeTable.put(2.0, 0.60);
flightTimeTable.put(3.0, 0.75);
flightTimeTable.put(4.0, 0.90);
// ... etc, from testing or calculation
```

### 6.3 Hood Angle Table (if applicable)
```java
// distance (meters) → hood angle (degrees)
InterpolatingTreeMap<Double, Double> hoodAngleTable = new InterpolatingTreeMap<>();
```

---

## 7. Vision & Localization with Limelight

### AprilTag-Based Pose Estimation

The Limelight can detect AprilTags and provide:
- **Robot pose** on the field (MegaTag/MegaTag2)
- **Distance and angle** to specific tags

### Recommended Architecture

1. **Primary localization**: Swerve odometry fused with Limelight AprilTag pose estimates using WPILib's `SwerveDrivePoseEstimator`.
2. **Hub detection**: Use known AprilTag positions near the hub to compute distance/angle to the hub center.
3. **Latency compensation**: Limelight has ~20-50ms of latency. Use the timestamp from the Limelight to add the vision measurement at the correct time in the pose estimator.

### Pose From Vision (1706's Method)

1706 also computed pose directly from Limelight + turret angle + gyro:
```java
double totalAngle = gyroRadians + turretAngle + limelightTx;
double robotX = hubX - limelightDistance * Math.cos(totalAngle);
double robotY = hubY - limelightDistance * Math.sin(totalAngle);
```

This provides a secondary pose estimate that can be fused with odometry.

### Our Existing Infrastructure

We already have:
- `LimelightSubsystem` with AprilTag detection
- `LimelightHelpers` utility class
- `DriveToAprilTagCommand` for navigation
- Swerve odometry via YAGSL

We can leverage all of this for the shooting system.

---

## 8. Turret Aiming Strategy

### Static Aiming (What We Have Now)
- Turret points at a fixed angle commanded by the operator.

### Auto-Aim at Hub (Next Step)
- Use robot pose + known hub position to calculate the angle from the robot to the hub.
- Subtract the robot's heading to get the turret angle in robot-relative coordinates.
- Command the turret to that angle.

```java
Translation2d robotToHub = hubPosition.minus(robotPose.getTranslation());
double fieldAngleToHub = Math.atan2(robotToHub.getY(), robotToHub.getX());
double turretAngle = fieldAngleToHub - robotPose.getRotation().getRadians();
turretSubsystem.setTargetAngleDegrees(Math.toDegrees(turretAngle));
```

### Shoot-While-Moving Aiming (Full Implementation)
- Same as above, but aim at the **virtual goal** instead of the real hub.
- The turret tracks the virtual goal continuously.

---

## 9. Implementation Plan

### Phase 1: Static Shooting (Foundation)
- [x] Turret position control with MAXMotion
- [x] Slow zone near limits
- [ ] Build a `ShooterSubsystem` (flywheel motor + speed control)
- [ ] Empirical testing: build RPM, flight time, and (optionally) hood angle tables
- [ ] Auto-aim command: turret tracks hub using robot pose

### Phase 2: Shoot While Moving
- [ ] Create `ShootingCalculations` utility class:
  - Virtual goal computation
  - Flight time lookup
  - RPM lookup
  - Turret angle calculation
- [ ] Create `ShootWhileMoveCommand`:
  - Continuously updates virtual goal from robot velocity
  - Commands turret angle, flywheel RPM
  - Fires when turret is on-target and flywheel is at speed
- [ ] Add acceleration compensation factor (tune empirically)
- [ ] Add iterative flight-time refinement (2-3 iterations)

### Phase 3: Polish & Tuning
- [ ] Latency compensation for Limelight measurements
- [ ] "Ready to shoot" indicator (LED, dashboard, rumble)
- [ ] Driver rumble when turret is in deadzone (can't aim at hub)
- [ ] Autonomous shooting routines
- [ ] Tune kAccelCompFactor on real robot

---

## 10. Suggestions & Recommendations

### Start Simple
1. **Get static shooting working first.** Nail the lookup tables at 3-5 known distances. Interpolation handles the rest.
2. **Add auto-aim second.** Make the turret track the hub using pose estimation. This alone is a huge competitive advantage.
3. **Add shoot-while-move last.** It's the cherry on top, but it requires everything else to be solid.

### Use WPILib Built-Ins
- `InterpolatingTreeMap` for all lookup tables (replaces custom LinearInterpolationTable)
- `SwerveDrivePoseEstimator` for fusing odometry + vision (YAGSL already does this)
- `ChassisSpeeds` for robot velocity (already available from swerve drive)

### Mechanical Considerations
- **Flywheel inertia matters.** A heavier flywheel maintains speed better between shots but takes longer to spin up. Use the [ReCalc flywheel tool](https://www.reca.lc/flywheel) to optimize.
- **Ball compression/slip** between shooter wheels significantly affects exit velocity. The [FRC 2026 Shooter Tuning Tool](https://ramalamadingdong.github.io/frc-shooter-selector) can help estimate this.
- **Consistent ball feeding** is critical. Inconsistent feed timing adds error that no amount of software can fix.

### Error Budget
- The [Shot Planner 2026](https://shotplanner2025.onrender.com/) tool shows the allowable error in angle and velocity for a given distance. Use it to understand how tight your tolerances need to be.
- At closer distances, you have more margin for error. Shooting from farther away requires much tighter control.
- The hub rim width provides a natural "error budget" for horizontal displacement.

### Latency is the Enemy
- Limelight: ~20-50ms latency
- SparkMax CAN bus: ~20ms round trip
- Code loop: 20ms
- Ball travel from feeder to shooter exit: variable
- **Total latency can be 60-100ms+**. The acceleration compensation factor helps, but minimizing latency at every stage pays dividends.

---

## 11. Key References

| Resource | Link | Notes |
|---|---|---|
| **1706 Shoot While Move Code** | [CD Thread](https://www.chiefdelphi.com/t/shoot-while-move-code-1706/410494) | Best real-world implementation reference. Virtual target method with accel compensation. |
| **Advice on Shooting While Moving** | [CD Thread](https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472) | Multiple approaches discussed: virtual target, velocity cancellation, full 3D physics. |
| **Shooting While Moving (Vectors)** | [CD Thread](https://www.chiefdelphi.com/t/shooting-while-moving/452828) | Vector subtraction approach, moving target interception math. |
| **Shooting While Driving / Good Code** | [CD Thread](https://www.chiefdelphi.com/t/shooting-while-driving-good-shooting-code/451451) | Iterative convergence, Spartronics quadratic interception code. |
| **Shot Planner 2026** | [Web App](https://shotplanner2025.onrender.com/) | Trajectory calculator with error budgets. Updated for 2026 hub dimensions. |
| **Desmos Trajectory Sim** | [Desmos](https://www.desmos.com/calculator/ten9e4rfzn) | Interactive 2D trajectory visualization (units in inches). |
| **FRC Shooter Tuning Tool** | [Web App](https://ramalamadingdong.github.io/frc-shooter-selector) | Two-wheel shooter optimization: RPM, spin-up, compression, drag. |
| **Finding Allowable Error** | [CD Thread](https://www.chiefdelphi.com/t/finding-allowable-error-in-shooting/399628) | Original 2022 error analysis tool, updated for 2026. |
| **All REBUILT Resources** | [CD Thread](https://www.chiefdelphi.com/t/all-the-rebuilt-resources/510081) | Master list of 2026 season resources. |
| **FRC 6302 TrackTargetLeadingPose** | [GitHub](https://github.com/FRC6302/2022Bot1/blob/master/src/main/java/frc/robot/commands/TrackTargetLeadingPose.java) | Example implementation of virtual target tracking. |
| **Team 5818 PhysicsUtil** | [GitHub](https://github.com/Team5818/CheeseWrangler-2020/blob/master/src/main/java/org/rivierarobotics/util/PhysicsUtil.java) | Full 3D physics calculations with arc/straight shot modes. |
| **WPILib InterpolatingTreeMap** | [GitHub](https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/src/main/java/edu/wpi/first/util/InterpolatingTreeMap.java) | Built-in interpolation table — use this instead of custom implementations. |
| **Ballistic Trajectory with Drag** | [CD Paper](https://www.chiefdelphi.com/t/paper-ballistic-trajectory-with-air-friction-drag-and-magnus/123764) | Advanced: adding drag and Magnus effect to trajectory calculations. |

---

## 12. Pseudocode — ShootWhileMoveCommand

```java
public class ShootWhileMoveCommand extends Command {

    // Subsystems
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;  // TODO: create this
    private final SwerveSubsystem drive;
    private final LimelightSubsystem limelight;

    // Known hub position on the field (meters, field coordinates)
    private static final Translation2d HUB_POSITION = new Translation2d(..., ...);

    // Lookup tables (populated from empirical testing)
    private final InterpolatingTreeMap<Double, Double> rpmTable;
    private final InterpolatingTreeMap<Double, Double> flightTimeTable;

    // Acceleration compensation factor (tune empirically, ~0.05-0.15 seconds)
    private static final double K_ACCEL_COMP = 0.1;

    // Number of iterations for flight-time refinement
    private static final int ITERATIONS = 3;

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();
        ChassisSpeeds fieldVel = drive.getFieldRelativeSpeeds();
        // TODO: get field-relative acceleration if available

        // Start with real hub position
        Translation2d virtualGoal = HUB_POSITION;

        // Iteratively refine the virtual goal
        for (int i = 0; i < ITERATIONS; i++) {
            double dist = robotPose.getTranslation().getDistance(virtualGoal);
            double flightTime = flightTimeTable.get(dist);

            double vgX = HUB_POSITION.getX()
                - flightTime * (fieldVel.vxMetersPerSecond /* + accelX * K_ACCEL_COMP */);
            double vgY = HUB_POSITION.getY()
                - flightTime * (fieldVel.vyMetersPerSecond /* + accelY * K_ACCEL_COMP */);

            virtualGoal = new Translation2d(vgX, vgY);
        }

        // Calculate turret angle to virtual goal
        Translation2d robotToVG = virtualGoal.minus(robotPose.getTranslation());
        double fieldAngle = Math.atan2(robotToVG.getY(), robotToVG.getX());
        double turretAngleDeg = Math.toDegrees(fieldAngle - robotPose.getRotation().getRadians());

        // Normalize to [-180, 180]
        turretAngleDeg = MathUtil.inputModulus(turretAngleDeg, -180, 180);

        // Calculate flywheel RPM for distance to virtual goal
        double distToVG = robotPose.getTranslation().getDistance(virtualGoal);
        double targetRPM = rpmTable.get(distToVG);

        // Command subsystems
        turret.setTargetAngleDegrees(turretAngleDeg);
        shooter.setTargetRPM(targetRPM);

        // Fire when ready
        if (turret.isOnTarget() && shooter.isAtSpeed()) {
            shooter.feed();  // release the ball into the shooter
        }
    }
}
```

---

*Document created: Feb 13, 2026*
*For Team 1647 — Iron Swerve*
