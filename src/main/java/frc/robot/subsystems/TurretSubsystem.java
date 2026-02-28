package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret subsystem using a Neo motor + SparkMax with onboard PID position control.
 * 
 * The SparkMax's internal encoder tracks motor rotations. With a 10:1 gear ratio,
 * 10 motor rotations = 1 turret rotation (360°). The onboard PID runs at 1kHz for
 * smooth, responsive position holding.
 * 
 * Uses MAXMotion position control for trapezoidal velocity profiling (30 RPM cruise,
 * smooth accel/decel). A software "slow zone" near the travel limits reduces output
 * and current so any contact with a physical hard stop is a gentle tap.
 * 
 * Hardware soft limits are configured as a safety backstop to prevent the turret
 * from exceeding ±170° even if software fails.
 * 
 * Assumes turret is at 0° (forward) on power-on.
 */
public class TurretSubsystem extends SubsystemBase
{

  // ── Constants ──────────────────────────────────────────────────────────────
  // CAN ID for the turret SparkMax — must match the physical controller
  public static final int TURRET_CAN_ID = 25;

  // Mechanical: 10 motor rotations per 1 turret rotation
  public static final double GEAR_RATIO = 10.0;

  // Turret travel limits in degrees (symmetric about 0°)
  public static final double MAX_ANGLE_DEGREES = 170.0;
  public static final double MIN_ANGLE_DEGREES = -170.0;

  // Soft limits in motor rotations (±170° converted through gear ratio)
  // 170° / 360° * 10 motor-rot/turret-rot = 4.7222 motor rotations
  public static final double SOFT_LIMIT_FORWARD_ROTATIONS =
      (MAX_ANGLE_DEGREES / 360.0) * GEAR_RATIO;
  public static final double SOFT_LIMIT_REVERSE_ROTATIONS =
      (MIN_ANGLE_DEGREES / 360.0) * GEAR_RATIO;

  // SparkMax onboard PID gains for position control (Slot 0)
  // Start conservative — tune on the real robot
  public static final double kP  = 0.1;
  public static final double kI  = 0.0;
  public static final double kD  = 0.01;
  public static final double kFF = 0.0;

  // Current limit (amps) to protect the motor and gearbox
  public static final int CURRENT_LIMIT_AMPS = 30;

  // Joystick deadband to ignore stick noise
  public static final double JOYSTICK_DEADBAND = 0.08;

  // Max motor speed in RPM — limits the NEO to 1 revolution per 2 seconds
  public static final double MAX_MOTOR_RPM = 30.0;

  // Max turret speed in degrees/second when joystick is fully deflected
  // At 50Hz periodic, each tick moves up to MAX_SPEED_DEG_PER_SEC / 50 degrees
  public static final double MAX_SPEED_DEG_PER_SEC = 90.0;

  // Button-press rotation speed (degrees per 20ms tick)
  public static final double BUTTON_SPEED_DEG_PER_TICK = 1.5;

  // ── Slow Zone Constants ──────────────────────────────────────────────────
  // When the turret is within SLOW_ZONE_DEGREES of either limit, output and
  // current are linearly ramped down so that contact with a physical hard stop
  // is a gentle tap rather than a slam.
  public static final double SLOW_ZONE_DEGREES = 20.0;

  // Normal PID output range (symmetric)
  public static final double NORMAL_OUTPUT_LIMIT = 0.5;

  // Minimum PID output at the very edge of the travel range
  public static final double MIN_OUTPUT_LIMIT = 0.08;

  // Reduced current limit (amps) when inside the slow zone
  public static final int SLOW_ZONE_CURRENT_LIMIT_AMPS = 10;

  // ── Hardware ───────────────────────────────────────────────────────────────
  private final SparkMax                  turretMotor;
  private final RelativeEncoder           turretEncoder;
  private final SparkClosedLoopController turretPID;

  // ── State ──────────────────────────────────────────────────────────────────
  // Target position in motor rotations — the PID setpoint
  private double targetMotorRotations = 0.0;

  // Tracks whether we are currently in the slow zone so we only reconfigure
  // the SparkMax when transitioning in or out (avoids CAN bus spam).
  private boolean inSlowZone = false;

  // Reusable config objects for dynamic reconfiguration
  private final SparkMaxConfig slowZoneConfig = new SparkMaxConfig();
  private final SparkMaxConfig normalConfig   = new SparkMaxConfig();

  public TurretSubsystem()
  {
    turretMotor = new SparkMax(TURRET_CAN_ID, MotorType.kBrushless);
    turretMotor.setCANTimeout(250);

    turretEncoder = turretMotor.getEncoder();
    turretPID = turretMotor.getClosedLoopController();

    // ── Build configuration ────────────────────────────────────────────────
    SparkMaxConfig config = new SparkMaxConfig();

    // Brake mode so turret holds position when disabled
    config.idleMode(IdleMode.kBrake);
    config.voltageCompensation(12.0);
    config.smartCurrentLimit(CURRENT_LIMIT_AMPS);

    // Onboard PID gains (Slot 0)
    ClosedLoopConfig pidConfig = config.closedLoop;
    pidConfig.p(kP);
    pidConfig.i(kI);
    pidConfig.d(kD);
    // TODO: ff() was removed in REVLib 2026. To add feedforward in the future,
    // use turretPID.setReference(setpoint, ControlType.kPosition, 0, kFF, ArbFFUnits.kVoltage)
    // pidConfig.ff(kFF);
    pidConfig.outputRange(-0.5, 0.5); // Limit max output for safety during tuning
    pidConfig.maxMotion.cruiseVelocity(MAX_MOTOR_RPM); // Limit NEO to 30 RPM (1 rev per 2 sec)
    pidConfig.maxMotion.maxAcceleration(MAX_MOTOR_RPM * 2); // Reach max speed in 0.5 sec

    // Hardware soft limits — the SparkMax enforces these in firmware
    SoftLimitConfig softLimits = config.softLimit;
    softLimits.forwardSoftLimitEnabled(true);
    softLimits.forwardSoftLimit(SOFT_LIMIT_FORWARD_ROTATIONS);
    softLimits.reverseSoftLimitEnabled(true);
    softLimits.reverseSoftLimit(SOFT_LIMIT_REVERSE_ROTATIONS);

    // Apply config — reset to safe defaults first, then persist to flash
    turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Encoder starts at 0 on power-on (turret assumed at 0°)
    turretEncoder.setPosition(0.0);
    targetMotorRotations = 0.0;

    // Pre-build the two configs used for dynamic slow-zone switching.
    // These only override output range and current limit; all other settings
    // are inherited from the initial config already persisted to flash.
    normalConfig.smartCurrentLimit(CURRENT_LIMIT_AMPS);
    normalConfig.closedLoop.outputRange(-NORMAL_OUTPUT_LIMIT, NORMAL_OUTPUT_LIMIT);

    slowZoneConfig.smartCurrentLimit(SLOW_ZONE_CURRENT_LIMIT_AMPS);
    slowZoneConfig.closedLoop.outputRange(-MIN_OUTPUT_LIMIT, MIN_OUTPUT_LIMIT);
  }

  // ── Public API ─────────────────────────────────────────────────────────────

  /**
   * Move the turret to an absolute angle in degrees.
   * Clamped to [MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES].
   */
  public void setTargetAngleDegrees(double angleDegrees)
  {
    angleDegrees = clampAngle(angleDegrees);
    targetMotorRotations = degreesToMotorRotations(angleDegrees);
    turretPID.setSetpoint(targetMotorRotations, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Adjust the turret position by a delta in degrees.
   * The resulting angle is clamped to the safe range.
   */
  public void adjustAngleDegrees(double deltaDegrees)
  {
    double currentTargetDeg = motorRotationsToDegrees(targetMotorRotations);
    setTargetAngleDegrees(currentTargetDeg + deltaDegrees);
  }

  /**
   * Command the turret to center (0°).
   */
  public void centerTurret()
  {
    setTargetAngleDegrees(0.0);
  }

  /**
   * Stop the motor (no PID, coast to stop). Use for emergencies.
   */
  public void stop()
  {
    turretMotor.stopMotor();
  }

  /**
   * Get the current turret angle in degrees based on the encoder.
   */
  public double getCurrentAngleDegrees()
  {
    return motorRotationsToDegrees(turretEncoder.getPosition());
  }

  /**
   * Get the current PID target angle in degrees.
   */
  public double getTargetAngleDegrees()
  {
    return motorRotationsToDegrees(targetMotorRotations);
  }

  /**
   * Returns true if the turret is within 2° of its target.
   */
  public boolean isOnTarget()
  {
    return Math.abs(getCurrentAngleDegrees() - getTargetAngleDegrees()) < 2.0;
  }

  // ── Periodic ───────────────────────────────────────────────────────────────

  @Override
  public void periodic()
  {
    double currentDeg = getCurrentAngleDegrees();

    // ── Slow-zone management ─────────────────────────────────────────────
    // Check how close we are to either travel limit
    double distToLimit = Math.min(
        Math.abs(currentDeg - MIN_ANGLE_DEGREES),
        Math.abs(currentDeg - MAX_ANGLE_DEGREES));
    boolean shouldBeInSlowZone = distToLimit < SLOW_ZONE_DEGREES;

    // Only reconfigure on transitions to avoid CAN bus spam
    if (shouldBeInSlowZone && !inSlowZone)
    {
      turretMotor.configure(slowZoneConfig,
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      inSlowZone = true;
    } else if (!shouldBeInSlowZone && inSlowZone)
    {
      turretMotor.configure(normalConfig,
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      inSlowZone = false;
    }

    // ── Telemetry ────────────────────────────────────────────────────────
    SmartDashboard.putNumber("Turret/CurrentDeg", currentDeg);
    SmartDashboard.putNumber("Turret/TargetDeg", getTargetAngleDegrees());
    SmartDashboard.putNumber("Turret/MotorRotations", turretEncoder.getPosition());
    SmartDashboard.putBoolean("Turret/OnTarget", isOnTarget());
    SmartDashboard.putNumber("Turret/MotorCurrent", turretMotor.getOutputCurrent());
    SmartDashboard.putNumber("Turret/DistToLimit", distToLimit);
    SmartDashboard.putBoolean("Turret/InSlowZone", inSlowZone);
  }

  // ── Helpers ────────────────────────────────────────────────────────────────

  private double degreesToMotorRotations(double degrees)
  {
    return (degrees / 360.0) * GEAR_RATIO;
  }

  private double motorRotationsToDegrees(double motorRotations)
  {
    return (motorRotations / GEAR_RATIO) * 360.0;
  }

  private double clampAngle(double degrees)
  {
    return Math.max(MIN_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, degrees));
  }
}
