package frc.robot;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;

import static edu.wpi.first.units.Units.*;

public class Limelight {
    private static NetworkTable limelightTable;

    static {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void setTargetID(int id) {
        limelightTable.getEntry("priorityid").setInteger(id);
    }

    public static boolean hasTarget() {
        return limelightTable.getEntry("tv").getInteger(0) != 0;
    }

    public static OptionalDouble getTargetX() {
        return hasTarget() ? OptionalDouble.of(limelightTable.getEntry("tx").getDouble(0.0)) : OptionalDouble.empty(); 
    }

    public static OptionalDouble getTargetY() {
        return hasTarget() ? OptionalDouble.of(limelightTable.getEntry("tx").getDouble(0.0)) : OptionalDouble.empty(); 
    }

    public static OptionalDouble getTargetArea() {
        return hasTarget() ? OptionalDouble.of(limelightTable.getEntry("ta").getDouble(0.0)) : OptionalDouble.empty(); 
    }

    public void setTargetID() {
        setTargetID(0);
    }

    public OptionalInt getTargetID() {
        return hasTarget() ? OptionalInt.of((int)limelightTable.getEntry("tid").getInteger(0)) : OptionalInt.empty();
    }


    public Twist3d getCameraOffset() {
        double[] poseArray = limelightTable.getEntry("camerapose_robotspace").getDoubleArray(new double[6]);
        return new Twist3d(
            poseArray[0], 
            poseArray[1], 
            poseArray[2], 
            Radians.convertFrom(poseArray[3], Degrees), 
            Radians.convertFrom(poseArray[4], Degrees), 
            Radians.convertFrom(poseArray[5], Degrees)
        );
    }

    public void setCameraOffset(Twist3d offset) {
        limelightTable.getEntry("camerapose_robotspace_set").setDoubleArray(new double[] {
            offset.dx,
            offset.dy,
            offset.dz,
            Degrees.convertFrom(offset.rx, Radians),
            Degrees.convertFrom(offset.ry, Radians),
            Degrees.convertFrom(offset.rz, Radians)
        });
    }

    public static Optional<Pose2d> getFieldCentricRobotPose(DistanceUnit distanceUnit, boolean useMegaTag2) {
        if (hasTarget()) {
            double[] poseArray = useMegaTag2 ? limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]) : limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            return Optional.of(new Pose2d(distanceUnit.convertFrom(poseArray[0], Meters), distanceUnit.convertFrom(poseArray[1], Meters), new Rotation2d(Radians.convertFrom(poseArray[5], Degrees))));
        } else {
            return Optional.empty();
        }
    }

    public static void setRobotYaw(double angle, double angularVelocity, AngleUnit angleUnit, AngularVelocityUnit angularVelocityUnit) {
        limelightTable.getEntry("robot_orientation_set").setDoubleArray(new double[] {
            Degrees.convertFrom(angle, angleUnit),
            DegreesPerSecond.convertFrom(angularVelocity, angularVelocityUnit),
            0.0,
            0.0,
            0.0,
            0.0
        });
    }

    // private Pose2d getRobotSpaceTargetPose(DistanceUnit distanceUnit) {
    //     double[] poseArray = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    //     Pose2d aprilTagPose = new Pose2d(distanceUnit.convertFrom(poseArray[0], Meters), distanceUnit.convertFrom(poseArray[1], Meters), new Rotation2d(Radians.convertFrom(poseArray[4], Degrees)));
    //     return aprilTagPose;
    // }

    private static double[] getPoseArray() {
        double[] poseArray = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        return poseArray;
    }

    private static Pose2d getFieldCentricTagPose(Pose2d currentPose, DistanceUnit distanceUnit) {
        double[] poseArray = getPoseArray();
        Transform2d transformationToTag = new Transform2d(distanceUnit.convertFrom(poseArray[0], Meters), distanceUnit.convertFrom(poseArray[1], Meters), new Rotation2d(Radians.convertFrom(poseArray[4], Degrees)));
        Pose2d aprilTagPose = currentPose.plus(transformationToTag);
        return aprilTagPose;
    }

    public static Optional<Pose2d> getScoringPose(Pose2d currentPose, Transform2d offset, DistanceUnit distanceUnit) {
        if (hasTarget()) {
            Pose2d aprilTagPose = getFieldCentricTagPose(currentPose, distanceUnit);
            Pose2d targetPose = aprilTagPose.plus(offset);
            return Optional.of(targetPose);
        } else {
            return Optional.empty();
        }
    }
}
