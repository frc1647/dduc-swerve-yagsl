// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

/**
 * Subsystem for managing the Limelight 2+ camera and AprilTag detection.
 */
public class LimelightSubsystem extends SubsystemBase {

    private final String limelightName;
    private boolean hasTarget = false;
    private int currentTargetId = -1;
    private Pose2d lastKnownRobotPose = new Pose2d();
    private double lastTargetTimestamp = 0;

    /**
     * Creates a new LimelightSubsystem.
     */
    public LimelightSubsystem() {
        this.limelightName = LimelightConstants.LIMELIGHT_NAME;

        // Configure the camera pose relative to robot center
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            LimelightConstants.CAMERA_FORWARD_OFFSET_METERS,  // forward
            LimelightConstants.CAMERA_SIDE_OFFSET_METERS,     // side (0 = centered)
            LimelightConstants.CAMERA_UP_OFFSET_METERS,       // up
            LimelightConstants.CAMERA_ROLL_DEGREES,           // roll
            LimelightConstants.CAMERA_PITCH_DEGREES,          // pitch
            LimelightConstants.CAMERA_YAW_DEGREES             // yaw
        );

        // Set up the camera stream for the driver station
        // The Limelight streams video at http://limelight.local:5800
        // This adds it to CameraServer so it appears in Shuffleboard/SmartDashboard
        try {
            HttpCamera limelightFeed = new HttpCamera(
                "Limelight",
                "http://" + limelightName + ".local:5800/stream.mjpg",
                HttpCameraKind.kMJPGStreamer
            );
            CameraServer.startAutomaticCapture(limelightFeed);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to start Limelight camera feed: " + e.getMessage(), false);
        }

        // Initialize SmartDashboard entries for tag selection
        SmartDashboard.putNumber("Limelight/PrimaryTagID", 1);
        SmartDashboard.putNumber("Limelight/SecondaryTagID", -1);  // -1 means no secondary tag
        SmartDashboard.putString("Limelight/Status", "Initialized");
    }

    @Override
    public void periodic() {
        // Update target status
        hasTarget = LimelightHelpers.getTV(limelightName);

        if (hasTarget) {
            currentTargetId = (int) LimelightHelpers.getFiducialID(limelightName);
            lastTargetTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }

        // Publish telemetry to SmartDashboard
        SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget);
        SmartDashboard.putNumber("Limelight/CurrentTargetID", currentTargetId);
        SmartDashboard.putNumber("Limelight/TX", LimelightHelpers.getTX(limelightName));
        SmartDashboard.putNumber("Limelight/TY", LimelightHelpers.getTY(limelightName));
        SmartDashboard.putNumber("Limelight/TA", LimelightHelpers.getTA(limelightName));
    }

    /**
     * Gets the primary AprilTag ID to target from SmartDashboard.
     * @return The primary tag ID
     */
    public int getPrimaryTagId() {
        return (int) SmartDashboard.getNumber("Limelight/PrimaryTagID", 1);
    }

    /**
     * Gets the secondary AprilTag ID for triangulation from SmartDashboard.
     * @return The secondary tag ID, or -1 if not set
     */
    public int getSecondaryTagId() {
        return (int) SmartDashboard.getNumber("Limelight/SecondaryTagID", -1);
    }

    /**
     * Checks if the Limelight currently sees a valid target.
     * @return true if a target is visible
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    /**
     * Checks if a specific AprilTag is currently visible.
     * @param tagId The tag ID to check for
     * @return true if the specified tag is visible
     */
    public boolean canSeeTag(int tagId) {
        if (!hasTarget) {
            return false;
        }

        // Check raw fiducials for the specific tag
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
        for (RawFiducial fiducial : fiducials) {
            if (fiducial.id == tagId) {
                return true;
            }
        }
        return false;
    }

    /**
     * Gets the horizontal offset to the target in degrees.
     * @return TX value in degrees (positive = target is to the right)
     */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    /**
     * Gets the vertical offset to the target in degrees.
     * @return TY value in degrees (positive = target is above crosshair)
     */
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    /**
     * Gets the target area as a percentage of the image.
     * @return Target area (0-100)
     */
    public double getTA() {
        return LimelightHelpers.getTA(limelightName);
    }

    /**
     * Gets the current fiducial/AprilTag ID being tracked.
     * @return The fiducial ID, or -1 if no target
     */
    public int getCurrentTargetId() {
        return hasTarget ? currentTargetId : -1;
    }

    /**
     * Gets the robot's pose estimate from the Limelight using MegaTag.
     * Uses alliance color to determine coordinate system.
     * @return PoseEstimate containing robot pose and metadata
     */
    public PoseEstimate getBotPoseEstimate() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    /**
     * Gets the robot's 2D pose from the Limelight.
     * @return Pose2d of the robot, or empty pose if no valid data
     */
    public Pose2d getRobotPose() {
        var alliance = DriverStation.getAlliance();
        Pose2d pose;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            pose = LimelightHelpers.getBotPose2d_wpiRed(limelightName);
        } else {
            pose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
        }

        // Update last known pose if we have a valid reading
        if (hasTarget && pose.getX() != 0 && pose.getY() != 0) {
            lastKnownRobotPose = pose;
        }

        return pose;
    }

    /**
     * Gets the last known robot pose (useful for dead reckoning when target is lost).
     * @return The last valid robot pose from vision
     */
    public Pose2d getLastKnownRobotPose() {
        return lastKnownRobotPose;
    }

    /**
     * Gets the time since the target was last seen.
     * @return Time in seconds since last target acquisition
     */
    public double getTimeSinceLastTarget() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - lastTargetTimestamp;
    }

    /**
     * Gets the target pose in robot space (relative to robot center).
     * @return Pose3d of the target relative to the robot
     */
    public Pose3d getTargetPoseRobotSpace() {
        return LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);
    }

    /**
     * Calculates the pose that is a specified distance in front of an AprilTag.
     * The pose will be along the tag's perpendicular (facing the tag).
     * 
     * @param tagPose The pose of the AprilTag on the field
     * @param distanceMeters The distance in front of the tag (positive = in front)
     * @return The target pose for the robot
     */
    public static Pose2d calculatePoseInFrontOfTag(Pose2d tagPose, double distanceMeters) {
        // The tag's rotation points outward from the tag face
        // We want to be in front of the tag, facing it
        Rotation2d tagRotation = tagPose.getRotation();

        // Calculate the position that is distanceMeters in front of the tag
        // "In front" means along the tag's facing direction (opposite of tag normal)
        double offsetX = distanceMeters * tagRotation.getCos();
        double offsetY = distanceMeters * tagRotation.getSin();

        Translation2d targetPosition = new Translation2d(
            tagPose.getX() + offsetX,
            tagPose.getY() + offsetY
        );

        // Robot should face the tag (opposite direction of tag's facing)
        Rotation2d robotRotation = tagRotation.plus(Rotation2d.fromDegrees(180));

        return new Pose2d(targetPosition, robotRotation);
    }

    /**
     * Sets the priority tag ID for the Limelight to focus on.
     * @param tagId The AprilTag ID to prioritize
     */
    public void setPriorityTag(int tagId) {
        LimelightHelpers.setPriorityTagID(limelightName, tagId);
    }

    /**
     * Sets the valid tag IDs that the Limelight should track.
     * @param tagIds Array of valid tag IDs
     */
    public void setValidTagIds(int[] tagIds) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, tagIds);
    }

    /**
     * Updates the robot orientation for MegaTag2 pose estimation.
     * Call this periodically with the robot's current heading.
     * @param yawDegrees Robot yaw in degrees (0 = facing red alliance wall)
     */
    public void updateRobotOrientation(double yawDegrees) {
        LimelightHelpers.SetRobotOrientation(limelightName, yawDegrees, 0, 0, 0, 0, 0);
    }

    /**
     * Sets the pipeline index on the Limelight.
     * @param index Pipeline index (0-9)
     */
    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(limelightName, index);
    }

    /**
     * Turns the LEDs on.
     */
    public void setLEDsOn() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    /**
     * Turns the LEDs off.
     */
    public void setLEDsOff() {
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
    }

    /**
     * Sets LEDs to be controlled by the pipeline.
     */
    public void setLEDsPipelineControl() {
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
    }

    /**
     * Updates the status string displayed on SmartDashboard.
     * @param status The status message to display
     */
    public void setStatus(String status) {
        SmartDashboard.putString("Limelight/Status", status);
    }

    /**
     * Gets the name of this Limelight.
     * @return The Limelight network table name
     */
    public String getName() {
        return limelightName;
    }
}
