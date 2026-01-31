// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Command to drive the robot to a position 10 feet in front of a specified AprilTag.
 * Uses a state machine to handle target acquisition, approach, and recovery if the target is lost.
 * 
 * States:
 * - SEEKING: Looking for the target AprilTag
 * - APPROACHING: Driving toward the target pose
 * - LOST_RECOVERING: Target lost, using dead reckoning to get close, then searching
 * - FINAL_APPROACH: Close to target, making fine adjustments
 * - COMPLETE: At destination
 */
public class DriveToAprilTagCommand extends Command {

    /**
     * State machine states for the drive-to-tag behavior.
     */
    public enum State {
        SEEKING,
        APPROACHING,
        LOST_RECOVERING,
        FINAL_APPROACH,
        COMPLETE
    }

    private final SwerveSubsystem drivebase;
    private final LimelightSubsystem limelight;

    private State currentState = State.SEEKING;
    private Pose2d targetPose = new Pose2d();
    private Pose2d lastKnownTargetPose = new Pose2d();
    private Pose2d poseWhenLost = new Pose2d();

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private final Timer stateTimer = new Timer();
    private final Timer lostTimer = new Timer();

    private int primaryTagId;
    private int secondaryTagId;
    private boolean useSecondaryTag = false;

    private static final double TARGET_DISTANCE_METERS = Units.feetToMeters(10.0);

    /**
     * Creates a new DriveToAprilTagCommand.
     * 
     * @param drivebase The swerve drive subsystem
     * @param limelight The limelight subsystem
     */
    public DriveToAprilTagCommand(SwerveSubsystem drivebase, LimelightSubsystem limelight) {
        this.drivebase = drivebase;
        this.limelight = limelight;

        // Initialize PID controllers with values from constants
        xController = new PIDController(
            LimelightConstants.DRIVE_TO_TAG_P,
            LimelightConstants.DRIVE_TO_TAG_I,
            LimelightConstants.DRIVE_TO_TAG_D
        );
        yController = new PIDController(
            LimelightConstants.DRIVE_TO_TAG_P,
            LimelightConstants.DRIVE_TO_TAG_I,
            LimelightConstants.DRIVE_TO_TAG_D
        );
        rotationController = new PIDController(
            LimelightConstants.ROTATION_P,
            LimelightConstants.ROTATION_I,
            LimelightConstants.ROTATION_D
        );

        // Enable continuous input for rotation (-180 to 180 degrees)
        rotationController.enableContinuousInput(-180, 180);

        // Set tolerances
        xController.setTolerance(LimelightConstants.POSITION_TOLERANCE_METERS);
        yController.setTolerance(LimelightConstants.POSITION_TOLERANCE_METERS);
        rotationController.setTolerance(LimelightConstants.ROTATION_TOLERANCE_DEGREES);

        addRequirements(drivebase, limelight);
    }

    @Override
    public void initialize() {
        // Get tag IDs from SmartDashboard
        primaryTagId = limelight.getPrimaryTagId();
        secondaryTagId = limelight.getSecondaryTagId();
        useSecondaryTag = secondaryTagId > 0;

        // Set up the Limelight to prioritize our target tag
        limelight.setPriorityTag(primaryTagId);

        // If using secondary tag for triangulation, set valid IDs
        if (useSecondaryTag) {
            limelight.setValidTagIds(new int[]{primaryTagId, secondaryTagId});
        }

        // Turn on LEDs for better tracking
        limelight.setLEDsOn();

        // Reset state
        currentState = State.SEEKING;
        stateTimer.reset();
        stateTimer.start();
        lostTimer.reset();

        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();

        limelight.setStatus("SEEKING tag " + primaryTagId);
        SmartDashboard.putString("DriveToTag/State", currentState.toString());
    }

    @Override
    public void execute() {
        // Update robot orientation for MegaTag2
        limelight.updateRobotOrientation(drivebase.getHeading().getDegrees());

        switch (currentState) {
            case SEEKING:
                executeSeekingState();
                break;
            case APPROACHING:
                executeApproachingState();
                break;
            case LOST_RECOVERING:
                executeLostRecoveringState();
                break;
            case FINAL_APPROACH:
                executeFinalApproachState();
                break;
            case COMPLETE:
                // Stop the robot
                drivebase.drive(new Translation2d(0, 0), 0, true);
                break;
        }

        // Update telemetry
        SmartDashboard.putString("DriveToTag/State", currentState.toString());
        SmartDashboard.putNumber("DriveToTag/TargetX", targetPose.getX());
        SmartDashboard.putNumber("DriveToTag/TargetY", targetPose.getY());
        SmartDashboard.putNumber("DriveToTag/TargetRot", targetPose.getRotation().getDegrees());
    }

    /**
     * SEEKING state: Rotate slowly to find the target tag.
     */
    private void executeSeekingState() {
        if (limelight.canSeeTag(primaryTagId)) {
            // Found the tag! Calculate target pose and transition to APPROACHING
            calculateTargetPose();
            transitionTo(State.APPROACHING);
            return;
        }

        // Rotate slowly to search for the tag
        drivebase.drive(new Translation2d(0, 0), LimelightConstants.SEARCH_ROTATION_SPEED, true);

        // Timeout check
        if (stateTimer.hasElapsed(LimelightConstants.SEEKING_TIMEOUT_SECONDS)) {
            limelight.setStatus("TIMEOUT: Tag " + primaryTagId + " not found");
            transitionTo(State.COMPLETE);
        }
    }

    /**
     * APPROACHING state: Drive toward the target pose while tracking the tag.
     */
    private void executeApproachingState() {
        // Check if we still see the tag
        if (!limelight.canSeeTag(primaryTagId)) {
            // Lost the tag - start recovery
            poseWhenLost = drivebase.getPose();
            lostTimer.reset();
            lostTimer.start();
            transitionTo(State.LOST_RECOVERING);
            return;
        }

        // Update target pose with fresh vision data
        calculateTargetPose();

        // Drive toward target
        ChassisSpeeds speeds = calculateDriveSpeeds();
        drivebase.driveFieldOriented(speeds);

        // Check if we're close enough for final approach
        if (isNearTarget(LimelightConstants.FINAL_APPROACH_DISTANCE_METERS)) {
            transitionTo(State.FINAL_APPROACH);
        }

        // Timeout check
        if (stateTimer.hasElapsed(LimelightConstants.APPROACH_TIMEOUT_SECONDS)) {
            limelight.setStatus("TIMEOUT: Approach took too long");
            transitionTo(State.COMPLETE);
        }
    }

    /**
     * LOST_RECOVERING state: Dead reckon toward last known target, then search.
     */
    private void executeLostRecoveringState() {
        // If we regain sight of the tag, go back to approaching
        if (limelight.canSeeTag(primaryTagId)) {
            calculateTargetPose();
            transitionTo(State.APPROACHING);
            return;
        }

        // Check if secondary tag is visible for triangulation
        if (useSecondaryTag && limelight.canSeeTag(secondaryTagId)) {
            // Use secondary tag to help localize
            // The pose estimate should still work with the secondary tag visible
            if (limelight.hasTarget()) {
                calculateTargetPose();
                transitionTo(State.APPROACHING);
                return;
            }
        }

        double timeLost = lostTimer.get();

        if (timeLost < LimelightConstants.DEAD_RECKONING_TIME_SECONDS) {
            // Phase 1: Dead reckon toward the last known target pose
            ChassisSpeeds speeds = calculateDriveSpeeds();
            // Reduce speed during dead reckoning for safety
            speeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond * 0.5,
                speeds.vyMetersPerSecond * 0.5,
                speeds.omegaRadiansPerSecond * 0.5
            );
            drivebase.driveFieldOriented(speeds);
            limelight.setStatus("DEAD RECKONING to target");
        } else {
            // Phase 2: Stop and rotate to search for the tag
            // Turn toward where we expect the tag to be
            Pose2d currentPose = drivebase.getPose();
            double angleToTarget = Math.atan2(
                lastKnownTargetPose.getY() - currentPose.getY(),
                lastKnownTargetPose.getX() - currentPose.getX()
            );

            double rotationError = Math.toDegrees(angleToTarget) - currentPose.getRotation().getDegrees();
            // Normalize to -180 to 180
            while (rotationError > 180) rotationError -= 360;
            while (rotationError < -180) rotationError += 360;

            double rotationSpeed = rotationController.calculate(0, -rotationError);
            rotationSpeed = clamp(rotationSpeed, -LimelightConstants.MAX_ROTATION_SPEED, 
                                  LimelightConstants.MAX_ROTATION_SPEED);

            drivebase.drive(new Translation2d(0, 0), rotationSpeed, true);
            limelight.setStatus("SEARCHING for tag " + primaryTagId);
        }

        // Timeout check for recovery
        if (lostTimer.hasElapsed(LimelightConstants.RECOVERY_TIMEOUT_SECONDS)) {
            limelight.setStatus("FAILED: Could not recover tag " + primaryTagId);
            transitionTo(State.COMPLETE);
        }
    }

    /**
     * FINAL_APPROACH state: Fine adjustments to reach exact target pose.
     */
    private void executeFinalApproachState() {
        // Check if we still see the tag
        if (!limelight.canSeeTag(primaryTagId)) {
            // At this close range, losing the tag is concerning but we can try to finish
            // using dead reckoning for a short time
            if (limelight.getTimeSinceLastTarget() > 0.5) {
                poseWhenLost = drivebase.getPose();
                lostTimer.reset();
                lostTimer.start();
                transitionTo(State.LOST_RECOVERING);
                return;
            }
        } else {
            // Update target pose with fresh vision data
            calculateTargetPose();
        }

        // Drive toward target with tighter control
        ChassisSpeeds speeds = calculateDriveSpeeds();
        drivebase.driveFieldOriented(speeds);

        // Check if we've reached the target
        if (isAtTarget()) {
            limelight.setStatus("COMPLETE: At target position");
            transitionTo(State.COMPLETE);
        }

        // Timeout check
        if (stateTimer.hasElapsed(LimelightConstants.FINAL_APPROACH_TIMEOUT_SECONDS)) {
            limelight.setStatus("TIMEOUT: Final approach took too long");
            transitionTo(State.COMPLETE);
        }
    }

    /**
     * Calculates the target pose (10 feet in front of the AprilTag).
     */
    private void calculateTargetPose() {
        // Get the robot's pose from vision
        Pose2d robotPose = limelight.getRobotPose();

        // Get the target pose in robot space
        var targetInRobotSpace = limelight.getTargetPoseRobotSpace();

        if (targetInRobotSpace.getX() == 0 && targetInRobotSpace.getY() == 0) {
            // No valid target data, keep last known
            return;
        }

        // Calculate the tag's field position based on robot pose and relative target position
        // The target pose in robot space gives us the vector from robot to tag
        double tagFieldX = robotPose.getX() + 
            targetInRobotSpace.getX() * robotPose.getRotation().getCos() -
            targetInRobotSpace.getY() * robotPose.getRotation().getSin();
        double tagFieldY = robotPose.getY() + 
            targetInRobotSpace.getX() * robotPose.getRotation().getSin() +
            targetInRobotSpace.getY() * robotPose.getRotation().getCos();

        // The tag's rotation (facing outward from the tag)
        Rotation2d tagRotation = Rotation2d.fromDegrees(
            robotPose.getRotation().getDegrees() + targetInRobotSpace.getRotation().getZ()
        );

        Pose2d tagPose = new Pose2d(tagFieldX, tagFieldY, tagRotation);

        // Calculate the pose 10 feet in front of the tag
        targetPose = LimelightSubsystem.calculatePoseInFrontOfTag(tagPose, TARGET_DISTANCE_METERS);
        lastKnownTargetPose = targetPose;
    }

    /**
     * Calculates the chassis speeds needed to drive to the target pose.
     */
    private ChassisSpeeds calculateDriveSpeeds() {
        Pose2d currentPose = drivebase.getPose();

        // Calculate errors
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double rotationError = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();

        // Normalize rotation error to -180 to 180
        while (rotationError > 180) rotationError -= 360;
        while (rotationError < -180) rotationError += 360;

        // Calculate PID outputs
        double xSpeed = xController.calculate(0, -xError);
        double ySpeed = yController.calculate(0, -yError);
        double rotationSpeed = rotationController.calculate(0, -rotationError);

        // Clamp speeds
        xSpeed = clamp(xSpeed, -LimelightConstants.MAX_DRIVE_SPEED, LimelightConstants.MAX_DRIVE_SPEED);
        ySpeed = clamp(ySpeed, -LimelightConstants.MAX_DRIVE_SPEED, LimelightConstants.MAX_DRIVE_SPEED);
        rotationSpeed = clamp(rotationSpeed, -LimelightConstants.MAX_ROTATION_SPEED, 
                              LimelightConstants.MAX_ROTATION_SPEED);

        return new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    }

    /**
     * Checks if the robot is near the target (within specified distance).
     */
    private boolean isNearTarget(double distanceMeters) {
        Pose2d currentPose = drivebase.getPose();
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        return distance < distanceMeters;
    }

    /**
     * Checks if the robot is at the target pose (within tolerances).
     */
    private boolean isAtTarget() {
        Pose2d currentPose = drivebase.getPose();

        double positionError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.abs(
            targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees()
        );
        // Normalize rotation error
        while (rotationError > 180) rotationError -= 360;
        rotationError = Math.abs(rotationError);

        return positionError < LimelightConstants.POSITION_TOLERANCE_METERS &&
               rotationError < LimelightConstants.ROTATION_TOLERANCE_DEGREES;
    }

    /**
     * Transitions to a new state.
     */
    private void transitionTo(State newState) {
        currentState = newState;
        stateTimer.reset();
        stateTimer.start();
        limelight.setStatus(newState.toString() + " - Tag " + primaryTagId);
    }

    /**
     * Clamps a value between min and max.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivebase.drive(new Translation2d(0, 0), 0, true);

        // Reset LEDs to pipeline control
        limelight.setLEDsPipelineControl();

        stateTimer.stop();
        lostTimer.stop();

        if (interrupted) {
            limelight.setStatus("INTERRUPTED");
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == State.COMPLETE;
    }

    /**
     * Gets the current state of the command.
     */
    public State getCurrentState() {
        return currentState;
    }
}
