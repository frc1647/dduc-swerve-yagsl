//LimelightHelpers v1.14 (REQUIRES LLOS 2026.0 OR LATER)
// Trimmed version with essential AprilTag functionality

package frc.robot;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * LimelightHelpers provides static methods for interfacing with Limelight vision cameras in FRC.
 */
public class LimelightHelpers {

    private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

    public static class RawFiducial {
        public int id = 0;
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;
        public double distToCamera = 0;
        public double distToRobot = 0;
        public double ambiguity = 0;

        public RawFiducial(int id, double txnc, double tync, double ta, 
                          double distToCamera, double distToRobot, double ambiguity) {
            this.id = id;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
        }
    }

    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;
        public RawFiducial[] rawFiducials;
        public boolean isMegaTag2;

        public PoseEstimate() {
            this.pose = new Pose2d();
            this.timestampSeconds = 0;
            this.latency = 0;
            this.tagCount = 0;
            this.tagSpan = 0;
            this.avgTagDist = 0;
            this.avgTagArea = 0;
            this.rawFiducials = new RawFiducial[]{};
            this.isMegaTag2 = false;
        }

        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
                           int tagCount, double tagSpan, double avgTagDist,
                           double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
            this.rawFiducials = rawFiducials;
            this.isMegaTag2 = isMegaTag2;
        }
    }

    static final String sanitizeName(String name) {
        if ("".equals(name) || name == null) {
            return "limelight";
        }
        return name;
    }

    public static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), 
                          Units.degreesToRadians(inData[4]),
                          Units.degreesToRadians(inData[5])));
    }

    public static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    private static double extractArrayEntry(double[] inData, int position) {
        if (inData.length < position + 1) {
            return 0;
        }
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName, boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = getLimelightDoubleArrayEntry(limelightName, entryName);
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;

        if (poseArray.length == 0) {
            return new PoseEstimate();
        }

        var pose = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int) extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;
        RawFiducial[] rawFiducials;

        if (poseArray.length != expectedTotalVals) {
            rawFiducials = new RawFiducial[0];
        } else {
            rawFiducials = new RawFiducial[tagCount];
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }
        return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, isMegaTag2);
    }

    public static RawFiducial[] getRawFiducials(String limelightName) {
        var entry = getLimelightNTTableEntry(limelightName, "rawfiducials");
        var rawFiducialArray = entry.getDoubleArray(new double[0]);
        int valsPerEntry = 7;
        if (rawFiducialArray.length % valsPerEntry != 0) {
            return new RawFiducial[0];
        }
        int numFiducials = rawFiducialArray.length / valsPerEntry;
        RawFiducial[] rawFiducials = new RawFiducial[numFiducials];
        for (int i = 0; i < numFiducials; i++) {
            int baseIndex = i * valsPerEntry;
            rawFiducials[i] = new RawFiducial(
                (int) extractArrayEntry(rawFiducialArray, baseIndex),
                extractArrayEntry(rawFiducialArray, baseIndex + 1),
                extractArrayEntry(rawFiducialArray, baseIndex + 2),
                extractArrayEntry(rawFiducialArray, baseIndex + 3),
                extractArrayEntry(rawFiducialArray, baseIndex + 4),
                extractArrayEntry(rawFiducialArray, baseIndex + 5),
                extractArrayEntry(rawFiducialArray, baseIndex + 6));
        }
        return rawFiducials;
    }

    public static Boolean validPoseEstimate(PoseEstimate pose) {
        return pose != null && pose.rawFiducials != null && pose.rawFiducials.length != 0;
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static void Flush() {
        NetworkTableInstance.getDefault().flush();
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
        String key = tableName + "/" + entryName;
        return doubleArrayEntries.computeIfAbsent(key, k -> {
            NetworkTable table = getLimelightNTTable(tableName);
            return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
        });
    }

    public static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    public static void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }

    public static boolean getTV(String limelightName) {
        return 1.0 == getLimelightNTDouble(limelightName, "tv");
    }

    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }

    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    public static Pose3d getBotPose3d(String limelightName) {
        return toPose3D(getLimelightNTDoubleArray(limelightName, "botpose"));
    }

    public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
        return toPose3D(getLimelightNTDoubleArray(limelightName, "botpose_wpiblue"));
    }

    public static Pose3d getBotPose3d_wpiRed(String limelightName) {
        return toPose3D(getLimelightNTDoubleArray(limelightName, "botpose_wpired"));
    }

    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {
        return toPose2D(getLimelightNTDoubleArray(limelightName, "botpose_wpiblue"));
    }

    public static Pose2d getBotPose2d_wpiRed(String limelightName) {
        return toPose2D(getLimelightNTDoubleArray(limelightName, "botpose_wpired"));
    }

    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue", false);
    }

    public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpired", false);
    }

    public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true);
    }

    public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpired", true);
    }

    public static Pose3d getTargetPose3d_RobotSpace(String limelightName) {
        return toPose3D(getLimelightNTDoubleArray(limelightName, "targetpose_robotspace"));
    }

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    public static void setPriorityTagID(String limelightName, int ID) {
        setLimelightNTDouble(limelightName, "priorityid", ID);
    }

    public static void setLEDMode_PipelineControl(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    public static void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    public static void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    public static void setStreamMode_Standard(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    public static void setCameraPose_RobotSpace(String limelightName, double forward, double side, 
                                                 double up, double roll, double pitch, double yaw) {
        double[] entries = new double[6];
        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    public static void SetRobotOrientation(String limelightName, double yaw, double yawRate,
                                           double pitch, double pitchRate,
                                           double roll, double rollRate) {
        double[] entries = new double[6];
        entries[0] = yaw;
        entries[1] = yawRate;
        entries[2] = pitch;
        entries[3] = pitchRate;
        entries[4] = roll;
        entries[5] = rollRate;
        setLimelightNTDoubleArray(limelightName, "robot_orientation_set", entries);
        Flush();
    }

    public static void SetFiducialIDFiltersOverride(String limelightName, int[] validIDs) {
        double[] validIDsDouble = new double[validIDs.length];
        for (int i = 0; i < validIDs.length; i++) {
            validIDsDouble[i] = validIDs[i];
        }
        setLimelightNTDoubleArray(limelightName, "fiducial_id_filters_set", validIDsDouble);
    }
}
