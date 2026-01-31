// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  /**
   * Constants for the Limelight 2+ camera and AprilTag navigation.
   */
  public static final class LimelightConstants
  {
    // Limelight network name
    public static final String LIMELIGHT_NAME = "limelight";

    // Camera mounting position relative to robot center (in meters)
    // Camera is mounted 18 inches in front of center, facing forward
    public static final double CAMERA_FORWARD_OFFSET_METERS = Units.inchesToMeters(18.0);
    public static final double CAMERA_SIDE_OFFSET_METERS = 0.0;  // Centered left-right
    public static final double CAMERA_UP_OFFSET_METERS = Units.inchesToMeters(12.0);  // Adjust based on actual mounting height

    // Camera mounting angles (in degrees)
    public static final double CAMERA_ROLL_DEGREES = 0.0;
    public static final double CAMERA_PITCH_DEGREES = 0.0;  // Adjust if camera is tilted up/down
    public static final double CAMERA_YAW_DEGREES = 0.0;    // 0 = facing forward

    // Drive-to-tag PID constants (tune these on the actual robot)
    public static final double DRIVE_TO_TAG_P = 1.5;
    public static final double DRIVE_TO_TAG_I = 0.0;
    public static final double DRIVE_TO_TAG_D = 0.1;

    // Rotation PID constants
    public static final double ROTATION_P = 0.05;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.005;

    // Position tolerances
    public static final double POSITION_TOLERANCE_METERS = 0.05;  // 5 cm
    public static final double ROTATION_TOLERANCE_DEGREES = 2.0;  // 2 degrees

    // Speed limits
    public static final double MAX_DRIVE_SPEED = 2.0;       // m/s
    public static final double MAX_ROTATION_SPEED = 2.0;    // rad/s
    public static final double SEARCH_ROTATION_SPEED = 0.5; // rad/s for seeking

    // State machine timeouts (in seconds)
    public static final double SEEKING_TIMEOUT_SECONDS = 10.0;
    public static final double APPROACH_TIMEOUT_SECONDS = 15.0;
    public static final double RECOVERY_TIMEOUT_SECONDS = 5.0;
    public static final double FINAL_APPROACH_TIMEOUT_SECONDS = 10.0;

    // Dead reckoning parameters
    public static final double DEAD_RECKONING_TIME_SECONDS = 1.5;  // Time to dead reckon before searching
    public static final double FINAL_APPROACH_DISTANCE_METERS = 1.0;  // Distance to switch to final approach
  }
}
