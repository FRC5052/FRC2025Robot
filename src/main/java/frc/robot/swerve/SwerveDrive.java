package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.stream.Collectors;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight;
import frc.robot.Robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.swerve.SwerveModule;

public class SwerveDrive implements Sendable {
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final SwerveModule[] modules;
    private final SwerveIMU imu;
    private Pose2d pose;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
    private Optional<Rotation2d> targetHeading = Optional.empty();
    private Field2d field = new Field2d();
    private FieldObject2d tagTarget;
    private boolean poseOverriden = false;
    private double maxDriveSpeed = 0.0;
    private double maxTurnSpeed = 0.0;
    private double maxDriveAccel = 0.0;
    private double maxTurnAccel = 0.0;
    private PIDController moduleDriveController = new PIDController(0, 0, 0), modulePivotController = new PIDController(0, 0, 0);
    private ProfiledPIDController headingController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    private boolean fieldCentricTargetSpeeds;
    private ChassisSpeeds actualSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] wheelStates;
    private final SwerveModuleState[] actualWheelStates;
    private final SwerveModulePosition[] wheelPositions;
    private final SwerveModulePosition[] wheelDeltaPositions;
    private Optional<PowerDistribution> powerDistribution = Optional.empty();
    private boolean enabled = true;

    public static Builder builder() {
        return new Builder();
    }

    // public static final Struct<SwerveDrive> struct = new Struct<SwerveDrive>() {
        
    // };

    public enum HeadingControlMode {
        // Value is to be interpreted as raw speed.
        kSpeedOnly,
        // Value is the new heading setpoint.
        kHeadingSet,
        // Value is the change in the heading.
        kHeadingChange,
        ;

        public boolean requiresControlLoop() {
            switch (this) {
                case kHeadingChange:
                case kHeadingSet:
                    return true;
                default:
                    return false;
                
            }
        }
    }
    
    /**
     * Constructs a new SwerveDrive object with the given initial pose, imu, and modules.
     * @param initialPose The Pose2d object representing the robot's starting position.
     * @param imu A SwerveIMU representing the robot's onboard IMU.
     * @param modules A list of SwerveModules representing the robot's swerve drive modules.
     */
    public SwerveDrive(Pose2d initialPose, SwerveIMU imu, SwerveModule... modules) {
        this.imu = imu;
        this.imu.calibrate();
        this.imu.resetHeading();
        this.modules = modules;
        this.pose = initialPose;
        Translation2d[] positions = new Translation2d[this.modules.length];

        this.wheelPositions = new SwerveModulePosition[this.modules.length];
        this.wheelDeltaPositions = new SwerveModulePosition[this.modules.length];
        this.actualWheelStates = new SwerveModuleState[this.modules.length];
        for (int i = 0; i < this.modules.length; i++) {
            positions[i] = this.modules[i].getModuleOffset();
            this.wheelPositions[i] = new SwerveModulePosition(0, Rotation2d.fromRadians(this.modules[i].getActualAngle(Radians)));
            this.actualWheelStates[i] = new SwerveModuleState();
            SmartDashboard.putData("swerveDrive/modules/module" + i, this.modules[i]);
        } 
        this.kinematics = new SwerveDriveKinematics(positions);
        this.odometry = new SwerveDriveOdometry(
            this.kinematics, 
            new Rotation2d(-this.imu.getHeading(Radians)), 
            this.wheelPositions, 
            this.pose
        );
        this.field.setRobotPose(this.pose);
        tagTarget = field.getObject("tagTarget");
        this.imu.resetHeading();
        SmartDashboard.putData("field", this.field);
        SmartDashboard.putData("swerveDrive/drivePID", this.moduleDriveController);
        SmartDashboard.putData("swerveDrive/pivotPID", this.modulePivotController);
        SmartDashboard.putData("swerveDrive", this);
    }

    public Optional<PowerDistribution> getPowerDistribution() {
        return this.powerDistribution;
    }

    public void setPowerDistribution(PowerDistribution powerDistribution) {
        this.powerDistribution = Optional.of(powerDistribution);
    }

    public void setPowerDistribution() {
        this.powerDistribution = Optional.empty();
    }

    /**
     * Returns one of the modules in this swerve drive, based on the given index.
     * @param index The index to retrieve the module from.
     * @return The SwerveModule at the given index.
     */
    public SwerveModule getSwerveModule(int index) {
        return this.modules[index];
    }

    /**
     * Returns the number of moduless in this swerve drive.
     * @return The number of modules.
     */
    public int getNumSwerveModules() {
        return this.modules.length;
    }

    /**
     * Returns this swerve drive's IMU.
     * @return This swerve drive's SwerveIMU.
     */
    public SwerveIMU getIMU() {
        return this.imu;
    }

    // /**
    //  * Configures whether this swerve drive should be field centric or not.
    //  * @param fieldCentric If this swerve drive should be field centric.
    //  */
    // public void setIsFieldCentric(boolean fieldCentric) {
    //     System.out.println("fieldcentric <- " + fieldCentric);
    //     this.fieldCentricTargetSpeeds = fieldCentric; // fieldCentric;
    // }

    // /**
    //  * Returns whether this swerve drive is field centric or not.
    //  * @return If this swerve drive is field centric.
    //  */
    // public boolean isFieldCentricTargetSpeeds() {
    //     return this.fieldCentricTargetSpeeds;
    // }

    /**
     * Configures this swerve drive's maximum translational velocity.
     * @param speed The new maximum velocity of this swerve drive, in the given units.
     * @param unit The unit of velocity to use to convert the new value.
     */
    public void setMaxDriveSpeed(double speed, LinearVelocityUnit unit) {
        this.maxDriveSpeed = unit.toBaseUnits(speed);
    }

    /**
     * Configures this swerve drive's maximum translational velocity.
     * @param speed The new maximum velocity of this swerve drive.
     */
    public void setMaxDriveSpeed(Measure<LinearVelocityUnit> speed) {
        this.maxDriveSpeed = speed.baseUnitMagnitude();
    }
    
    /**
     * Configures this swerve drive's maximum rotational velocity.
     * @param speed The new maximum angular velocity of this swerve drive, in the given units.
     * @param unit The unit of angular velocity to use to convert the new value.
     */
    public void setMaxTurnSpeed(double speed, AngularVelocityUnit unit) {
        this.maxTurnSpeed = unit.toBaseUnits(speed);
    }

    /**
     * Configures this swerve drive's maximum rotational velocity.
     * @param speed The new maximum angular velocity of this swerve drive.
     */
    public void setMaxTurnSpeed(Measure<AngularVelocityUnit> speed) {
        this.maxTurnSpeed = speed.baseUnitMagnitude();
    }

    /**
     * Configures this swerve drive's maximum translational acceleration.
     * @param accel The new maximum acceleration of this swerve drive, in the given units.
     * @param unit The unit of acceleration to use to convert the new value.
     */
    public void setMaxDriveAccel(double accel, LinearAccelerationUnit unit) {
        this.maxDriveAccel = unit.toBaseUnits(accel);
    }

    /**
     * Configures this swerve drive's maximum translational acceleration.
     * @param accel The new maximum acceleration of this swerve drive.
     */
    public void setMaxDriveAccel(Measure<LinearAccelerationUnit> accel) {
        this.maxDriveAccel = accel.baseUnitMagnitude();
    }
    
    /**
     * Configures this swerve drive's maximum rotational acceleration.
     * @param accel The new maximum angular acceleration of this swerve drive, in the given units.
     * @param unit The unit of angular acceleration to use to convert the new value.
     */
    public void setMaxTurnAccel(double accel, AngularAccelerationUnit unit) {
        this.maxTurnAccel = unit.toBaseUnits(accel);
    }

    /**
     * Configures this swerve drive's maximum rotational acceleration.
     * @param accel The new maximum angular acceleration of this swerve drive.
     */
    public void setMaxTurnAccel(Measure<AngularAccelerationUnit> accel) {
        this.maxTurnAccel = accel.baseUnitMagnitude();
    }

    /**
     * Returns this swerve drive's maximum translational velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The maximum velocity of this swerve drive, in the given units.
     */
    public double getMaxDriveSpeed(LinearVelocityUnit unit) {
        return unit.fromBaseUnits(this.maxDriveSpeed);
    }

    /**
     * Returns this swerve drive's maximum rotational velocity.
     * @param unit The unit of angular velocity to use to convert the value.
     * @return The maximum angular velocity of this swerve drive, in the given units.
     */
    public double getMaxTurnSpeed(AngularVelocityUnit unit) {
        return unit.fromBaseUnits(this.maxTurnSpeed);
    }


    /**
     * Returns this swerve drive's maximum translational acceleration.
     * @param unit The unit of acceleration to use to convert the value.
     * @return The maximum acceleration of this swerve drive, in the given units.
     */
    public double getMaxDriveAccel(LinearAccelerationUnit unit) {
        return unit.fromBaseUnits(this.maxDriveAccel);
    }

    /**
     * Returns this swerve drive's maximum rotational acceleration.
     * @param unit The unit of angular acceleration to use to convert the value.
     * @return The maximum angular acceleration of this swerve drive, in the given units.
     */
    public double getMaxTurnAccel(AngularAccelerationUnit unit) {
        return unit.fromBaseUnits(this.maxTurnAccel);
    }

    /**
     * Returns the X component of this swerve drive's current pose.
     * @param unit The unit of distance to use to convert the value.
     * @return The X position of this swerve drive on the field, in the given units.
     */
    public double getPoseX(DistanceUnit unit) {
        return unit.convertFrom(this.pose.getX(), Meters);
    }

    /**
     * Returns the Y component of this swerve drive's current pose.
     * @param unit The unit of distance to use to convert the value.
     * @return The Y position of this swerve drive on the field, in the given units.
     */
    public double getPoseY(DistanceUnit unit) {
        return unit.convertFrom(this.pose.getY(), Meters);
    }

    /**
     * Returns the translational component of this swerve drive's current pose.
     * @return A Translation2d representing this swerve drive's field position.
     */
    public Translation2d getPosePositionMeters() {
        return this.pose.getTranslation();
    }

    /**
     * Returns the theta (angular) component of this swerve drive's current pose.
     * @param unit The unit of angle to use to convert the value.
     * @return The heading of this swerve drive on the field, in the given units.
     */
    public double getPoseAngle(AngleUnit unit) {
        return unit.convertFrom(this.pose.getRotation().getRadians(), Radians);
    }

    /**
     * Returns the theta (angular) component of this swerve drive's current pose.
     * @return The heading of this swerve drive on the field.
     */
    public Rotation2d getPoseAngle() {
        return this.pose.getRotation();
    }

    /** 
     * Returns the target heading of this swerve drive, if present.
     * @param unit The unit of angle to use to convert the value.
     * @return The target heading value, in the given units, if present.
     */
    public OptionalDouble getTargetHeading(AngleUnit unit) {
        return this.targetHeading.isPresent() ? OptionalDouble.of(unit.convertFrom(this.targetHeading.get().getRadians(), Radians)) : OptionalDouble.empty();
    }

    /** 
     * Returns the target heading of this swerve drive, if present.
     * @return The target heading value, if present.
     */
    public Optional<Rotation2d> getTargetHeading() {
        return this.targetHeading;
    }

    /** 
     * Returns the actual, measured heading of this swerve drive, as reported by the IMU.
     * @param unit The unit of angle to use to convert the value.
     * @return The measured heading value, in the given units.
     */
    public double getActualHeading(AngleUnit unit) {
        return this.imu.getHeading(unit);
    }

    public void enable() {
        this.enabled = true;
    }

    public void disable() {
        this.enabled = false;
    }

    public boolean isEnabled() {
        return this.enabled;
    }

    /**
     * Sets the targeted drive speed to the given values.
     * @param x The desired X speed. The value should be normalized (-1 to 1).
     * @param y The desired Y speed. The value should be normalized (-1 to 1).
     * @param h The desired angular speed or heading, depending on the mode. The value should be normalized (-1 to 1).
     * @param mode The heading control mode to use. This will change how the h value is interpreted.
     * @param fieldCentric If present, overrides the current fieldCentric setting.
     */
    public void drive(double x, double y, double h, HeadingControlMode mode, boolean fieldCentric) {
        x = MathUtil.clamp(x, -1.0, 1.0);
        y = MathUtil.clamp(y, -1.0, 1.0);
        h = MathUtil.clamp(h, -1.0, 1.0);

        this.targetSpeeds.vxMetersPerSecond = this.maxDriveSpeed * x;
        this.targetSpeeds.vyMetersPerSecond = this.maxDriveSpeed * y; 

        this.fieldCentricTargetSpeeds = fieldCentric;

        switch (mode) {
            case kHeadingChange:
                // this.targetRSpeed.mut_replace(this.maxTurnSpeed.times(h));
                Rotation2d targetHeading = this.targetHeading.orElse(this.pose.getRotation());
                this.targetHeading = Optional.of(new Rotation2d(targetHeading.getRadians() + (this.maxTurnSpeed * h * Robot.kDefaultPeriod)));
                break;
            case kHeadingSet:
                this.targetHeading = Optional.of(Rotation2d.fromRotations(h));
                break;
            case kSpeedOnly:
                this.targetSpeeds.omegaRadiansPerSecond = this.maxTurnSpeed * h;
                this.targetHeading = Optional.empty();
            default:
                break;
            
        }


    }

    /**
     * Sets the targeted drive speed to the given values.
     * @param speeds The ChassisSpeeds object to apply.
     * @param fieldCentric Whether the speeds value is field centric, otherwise it is assumed to be robot centric.
     */
    public void drive(ChassisSpeeds speeds, boolean fieldCentric) {
        this.fieldCentricTargetSpeeds = fieldCentric;
        this.targetSpeeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -this.maxDriveSpeed, this.maxDriveSpeed);
        this.targetSpeeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -this.maxDriveSpeed, this.maxDriveSpeed);
        this.targetSpeeds.omegaRadiansPerSecond = MathUtil.clamp(speeds.omegaRadiansPerSecond, -this.maxTurnSpeed, this.maxTurnSpeed);
        this.targetHeading = Optional.empty();
    } 

    /**
     * Configures the drive PID constants for all swerve modules.
     * @param constants The constants to apply.
     */
    public void setModuleDrivePID(PIDConstants constants) {
        this.moduleDriveController = new PIDController(constants.kP, constants.kI, constants.kD);
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setDrivePID(Optional.of(constants)); 
        }
    }

    /**
     * Configures the pivot PID constants for all swerve modules.
     * @param constants The constants to apply.
     */
    public void setModulePivotPID(PIDConstants constants) {
        this.modulePivotController = new PIDController(constants.kP, constants.kI, constants.kD);
        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setPivotPID(Optional.of(constants)); 
        }
    }

    /**
     * Configures the heading PID constants for heading targeting.
     * @param constants The constants to apply.
     */
    public void setHeadingPID(PIDConstants constants) {
        this.headingController.setP(constants.kP);
        this.headingController.setI(constants.kI);
        this.headingController.setD(constants.kD);
        this.headingController.setConstraints(new Constraints(this.maxTurnSpeed, this.maxTurnAccel));
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the drive PID controller for all swerve modules.
     * @return The common drive PIDController.
     */
    public PIDController getModuleDrivePID() {
        return this.moduleDriveController;
    }

    /**
     * Returns the pivot PID controller for all swerve modules.
     * @return The common pivot PIDController.
     */
    public PIDController getModulePivotPID() {
        return this.modulePivotController;
    }

    /**
     * Returns the heading PID controller for heading targeting.
     * @return The common heading ProfiledPIDController.
     */
    public ProfiledPIDController getHeadingController() {
        return this.headingController;
    }

    /**
     * Resets the heading of this swerve drive such that the new reported heading is zero.
     */
    public void zeroHeading() {
        throw new UnsupportedOperationException("This shouldn't be called, check your implementation (and your life choices!)");
        // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
        //     this.imu.resetHeading();
        //     this.setPose(new Pose2d(new Translation2d(this.pose.getX(), this.pose.getY()), Rotation2d.kZero));
        //     this.targetHeading = this.targetHeading.map((x) -> Rotation2d.kZero);
        // } else {
        //     // this.imu.zeroHeading();
        //     this.imu.setHeadingOffset(this.imu.getRawHeading(Degree) + 180, Degree);
        //     this.setPose(new Pose2d(new Translation2d(this.pose.getX(), this.pose.getY()), Rotation2d.k180deg));
        //     this.targetHeading = this.targetHeading.map((x) -> Rotation2d.k180deg);
        // }
    }

    /** 
     * Sets the pose of this swerve drive.
     * @param pose The new pose to apply.
     */
    public void setPose(Pose2d pose) {
        this.pose = pose;
        this.poseOverriden = true;
    }

    /**
     * Returns the pose of this swerve drive.
     * @return The pose of this swerve drive, in meters.
     */
    public Pose2d getPose() {
        return this.pose;
    }

    /**
     * Returns the X component of this swerve drive's velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The X component of this swerve drive's velocity, in the given units.
     */
    public double getVelocityX(LinearVelocityUnit unit) {
        return unit.convertFrom(this.actualSpeeds.vxMetersPerSecond, MetersPerSecond);
    }

    /**
     * Returns the Y component of this swerve drive's velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The Y component of this swerve drive's velocity, in the given units.
     */
    public double getVelocityY(LinearVelocityUnit unit) {
        return unit.convertFrom(this.actualSpeeds.vyMetersPerSecond, MetersPerSecond);
    }

    /**
     * Returns the magnitude of this swerve drive's velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The magnitude of this swerve drive's velocity, in the given units.
     */
    public double getVelocityMagnitude(LinearVelocityUnit unit) {
        return Math.hypot(this.getVelocityX(unit), this.getVelocityY(unit));
    }

    /**
     * Returns the direction of this swerve drive's velocity.
     * @param unit The unit of velocity to use to convert the value.
     * @return The direction of this swerve drive's velocity, in the given units.
     */
    public double getVelocityDirection(AngleUnit unit) {
        return unit.convertFrom(Math.atan2(this.getVelocityY(MetersPerSecond), this.getVelocityX(MetersPerSecond)), Radians);
    }

    /**
     * Returns the theta component of this swerve drive's velocity.
     * @param unit The unit of angular velocity to use to convert the value.
     * @return The theta component of this swerve drive's velocity, in the given units.
     */
    public double getTurnVelocity(AngularVelocityUnit unit) {
        return unit.convertFrom(this.actualSpeeds.omegaRadiansPerSecond, RadiansPerSecond);
    } 

    /**
     * Returns the velocity of this swerve drive.
     * @return A ChassisSpeeds representing this swerve drive's velocity.
     */
    public ChassisSpeeds getActualSpeeds() {
        return this.actualSpeeds;
    }

    /**
     * Returns the field of this swerve drive.
     * @return A Field2d representing this swerve drive's field.
     */
    public Field2d getField() {
        return this.field;
    }

    /**
     * Updates the internal logic of this swerve drive. This should be called in a command or subsystem's periodic function.
     */
    public void update() {
        if (!this.isEnabled()) return;

        // System.out.println(getPose());
        // System.out.println(fieldCentric);
        // if (fieldCentric) {
        //     System.out.println("yo");
        // }

        this.speeds = new ChassisSpeeds(
            SwerveUtil.limitAccelAndSpeed(this.targetSpeeds.vxMetersPerSecond, this.speeds.vxMetersPerSecond, Robot.kDefaultPeriod, this.maxDriveSpeed, this.maxDriveAccel),
            SwerveUtil.limitAccelAndSpeed(this.targetSpeeds.vyMetersPerSecond, this.speeds.vyMetersPerSecond, Robot.kDefaultPeriod, this.maxDriveSpeed, this.maxDriveAccel), 
            SwerveUtil.limitAccelAndSpeed((this.targetHeading.isPresent() && !DriverStation.isAutonomousEnabled() ? 
                -this.headingController.calculate(this.pose.getRotation().getRadians(), this.targetHeading.get().getRadians())
                :
                this.targetSpeeds.omegaRadiansPerSecond
            ), this.speeds.omegaRadiansPerSecond, Robot.kDefaultPeriod, this.maxTurnSpeed, this.maxTurnAccel)
        );

        if (this.fieldCentricTargetSpeeds) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                this.wheelStates = this.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(this.speeds, this.pose.getRotation()));
            } else {
                this.wheelStates = this.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(this.speeds, this.pose.getRotation().plus(Rotation2d.k180deg)));
            }
        } else {
            this.wheelStates = this.kinematics.toSwerveModuleStates(this.speeds);
        }

        

        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setTargetState(this.wheelStates[i]);
            this.modules[i].update();
            this.wheelPositions[i] = this.modules[i].getActualPosition();
            this.wheelDeltaPositions[i] = this.modules[i].getActualDeltaPosition();
            // this.wheelPositions[i].angle = this.wheelPositions[i].angle.rotateBy(Rotation2d.k180deg);
            this.actualWheelStates[i] = this.modules[i].getActualState();
        }

        this.actualSpeeds = this.kinematics.toChassisSpeeds(this.actualWheelStates);

        if (this.poseOverriden) {
            this.odometry.resetPosition(
                new Rotation2d(this.imu.getHeading(Radians)), 
                this.wheelPositions, 
                this.pose
            );
            this.poseOverriden = false;
        } else {
            this.pose = this.odometry.update(
                new Rotation2d(this.imu.getHeading(Radians)), 
                this.wheelPositions    
            );
        }
        this.field.setRobotPose(this.pose);
        this.tagTarget.setPose(
            Limelight.getScoringPose(
                getPose(), 
                OperatorConstants.kScoreLeftOffset,
                Meter
            ).orElse(getPose())
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("imu/heading", () -> this.getPoseAngle(Degrees), null);
        builder.addDoubleProperty("imu/rawHeading", () -> -this.imu.getRawHeading(Degrees), null);
        builder.addDoubleProperty("imu/headingSetpoint", () -> -this.targetHeading.orElse(Rotation2d.kZero).getDegrees(), null);
        builder.addDoubleProperty("imu/magneticHeading", () -> this.imu.getCompassHeading(Degrees), null);
        builder.addDoubleProperty("imu/headingOffset", () -> -this.imu.getHeadingOffset(Degrees), (double offset) -> this.imu.setHeadingOffset(-offset, Degrees));
        builder.addDoubleProperty("imu/velocity/x", () -> this.getVelocityX(MetersPerSecond), null);
        builder.addDoubleProperty("imu/velocity/y", () -> this.getVelocityY(MetersPerSecond), null);
        builder.addDoubleProperty("imu/velocity/norm", () -> this.getVelocityMagnitude(MetersPerSecond), null);
        builder.addDoubleProperty("imu/velocity/angle", () -> this.getVelocityDirection(Degrees), null);
        builder.addDoubleProperty("powDist/totalCurrent", () -> this.getPowerDistribution().map((PowerDistribution powDist) -> powDist.getTotalCurrent()).orElse(0.0), null);
        builder.addDoubleProperty("powDist/totalEnergy", () -> this.getPowerDistribution().map((PowerDistribution powDist) -> powDist.getTotalCurrent()).orElse(0.0), null);
        builder.addDoubleProperty("powDist/voltage", () -> this.getPowerDistribution().map((PowerDistribution powDist) -> powDist.getVoltage()).orElse(0.0), null);
        builder.addDoubleArrayProperty("powDist/moduleCurrents", () -> this.getPowerDistribution().map((PowerDistribution powDist) -> {
            double[] vals = new double[powDist.getNumChannels()];
            for (int i = 0; i < vals.length; i++) {
                // vals[i] = powDist.getCurrent(i);
            }
            return vals;
        }).orElse(new double[0]), null);
        builder.addDoubleArrayProperty("modules/actualStates", () -> {
            double[] vals = new double[this.actualWheelStates.length << 1];
            for (int i = 0; i < this.actualWheelStates.length; i++) {
                vals[(i << 1)] = this.actualWheelStates[i].angle.getDegrees();
                vals[(i << 1) + 1] = this.actualWheelStates[i].speedMetersPerSecond;
            }
            return vals;
        }, null);
        
    }

    public static class Builder {
        private Optional<SwerveModule.Builder[]> modules = Optional.empty();
        private Optional<SwerveIMU.SwerveIMUBuilder> imu = Optional.empty();
        private Pose2d initialPose = new Pose2d();
        private Optional<PIDConstants> moduleDrivePID = Optional.empty();
        private Optional<PIDConstants> modulePivotPID = Optional.empty();
        private Optional<PIDConstants> headingPID = Optional.empty();
        private OptionalDouble maxDriveSpeed = OptionalDouble.empty();
        private OptionalDouble maxTurnSpeed = OptionalDouble.empty();
        private OptionalDouble maxDriveAccel = OptionalDouble.empty();
        private OptionalDouble maxTurnAccel = OptionalDouble.empty();

        public Builder withModules(SwerveModule.Builder... modules) {
            this.modules = Optional.of(modules);
            return this;
        }

        public Builder withIMU(SwerveIMU.SwerveIMUBuilder imu) {
            this.imu = Optional.of(imu);
            return this;
        }

        public Builder withInitialPose(Pose2d pose) {
            this.initialPose = pose;
            return this;
        }

        public Builder withModuleDrivePID(PIDConstants constants) {
            this.moduleDrivePID = Optional.of(constants);
            return this;
        }

        public Builder withModulePivotPID(PIDConstants constants) {
            this.modulePivotPID = Optional.of(constants);
            return this;
        }

        public Builder withHeadingPID(PIDConstants constants) {
            this.headingPID = Optional.of(constants);
            return this;
        }

        public Builder withMaxDriveSpeed(double speed, LinearVelocityUnit unit) {
            this.maxDriveSpeed = OptionalDouble.of(unit.toBaseUnits(speed));
            return this;
        }

        public Builder withMaxTurnSpeed(double speed, AngularVelocityUnit unit) {
            this.maxTurnSpeed = OptionalDouble.of(unit.toBaseUnits(speed));
            return this;
        }

        public Builder withMaxDriveAccel(double speed, LinearAccelerationUnit unit) {
            this.maxDriveAccel = OptionalDouble.of(unit.toBaseUnits(speed));
            return this;
        }

        public Builder withMaxTurnAccel(double speed, AngularAccelerationUnit unit) {
            this.maxTurnAccel = OptionalDouble.of(unit.toBaseUnits(speed));
            return this;
        }

        public void fromJSON(JsonNode json) {
            if (json.has("modules") && json.get("modules").isArray()) {
                JsonNode modulesJSON = json.get("modules");
                SwerveModule.Builder[] modules = new SwerveModule.Builder[modulesJSON.size()];
                for (int i = 0; i < modules.length; i++) {
                    if (modulesJSON.has(i) && modulesJSON.get(i).isObject()) {
                        modules[i] = new SwerveModule.Builder();
                        modules[i].fromJSON(modulesJSON.get(i));
                    }
                }
                this.withModules(modules);
            }
            if (json.has("imu") && json.get("imu").isObject()) {
                this.withIMU(SwerveIMU.builderFromJSON(json.get("imu")));
            }
            
            if (json.has("headingPID") && json.get("headingPID").isObject()) {
                JsonNode json_inner = json.get("headingPID");
                if (json_inner.has("p") && json_inner.get("p").isDouble() && json_inner.has("i") && json_inner.get("i").isDouble() && json_inner.has("d") && json_inner.get("d").isDouble()) {
                    this.withHeadingPID(new PIDConstants(json_inner.get("p").doubleValue(), json_inner.get("i").doubleValue(), json_inner.get("d").doubleValue()));
                }
            }

            if (json.has("maxDriveSpeed") && (json.get("maxDriveSpeed").isObject() || json.get("maxDriveSpeed").isDouble())) {
                JsonNode json_inner = json.get("maxDriveSpeed");
                LinearVelocityUnit unit = MetersPerSecond;
                if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.velocityFromName(json_inner.get("unit").textValue()), unit);
                    this.withMaxDriveSpeed(json_inner.get("value").doubleValue(), unit);
                } else {
                    this.withMaxDriveSpeed(json_inner.doubleValue(), unit);
                }
            }

            if (json.has("maxTurnSpeed") && (json.get("maxTurnSpeed").isObject() || json.get("maxTurnSpeed").isDouble())) {
                JsonNode json_inner = json.get("maxTurnSpeed");
                AngularVelocityUnit unit = RadiansPerSecond;
                if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.angularVelocityFromName(json_inner.get("unit").textValue()), unit);
                    this.withMaxTurnSpeed(json_inner.get("value").doubleValue(), unit);
                } else {
                    this.withMaxTurnSpeed(json_inner.doubleValue(), unit);
                }
            }

            if (json.has("maxDriveAccel") && (json.get("maxDriveAccel").isObject() || json.get("maxDriveAccel").isDouble())) {
                JsonNode json_inner = json.get("maxDriveAccel");
                LinearAccelerationUnit unit = MetersPerSecondPerSecond;
                if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.accelFromName(json_inner.get("unit").textValue()), unit);
                    this.withMaxDriveAccel(json_inner.get("value").doubleValue(), unit);
                } else {
                    this.withMaxDriveAccel(json_inner.doubleValue(), unit);
                }
            }

            if (json.has("maxTurnAccel") && (json.get("maxTurnAccel").isObject() || json.get("maxTurnAccel").isDouble())) {
                JsonNode json_inner = json.get("maxTurnAccel");
                AngularAccelerationUnit unit = RadiansPerSecond.per(Second);
                if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                    unit = Objects.requireNonNullElse(SwerveUtil.angularAccelFromName(json_inner.get("unit").textValue()), unit);
                    this.withMaxTurnAccel(json_inner.get("value").doubleValue(), unit);
                } else {
                    this.withMaxTurnAccel(json_inner.doubleValue(), unit);
                }
            }
        }

        public SwerveDrive build() {
            if (this.modules.isEmpty()) {
                throw new IllegalStateException("Modules field was empty");
            }
            if (this.imu.isEmpty()) {
                throw new IllegalStateException("IMU field was empty");
            }
            SwerveModule.Builder[] modules = this.modules.get();
            SwerveModule[] new_modules = new SwerveModule[modules.length];
            for (int i = 0; i < new_modules.length; i++) {
                new_modules[i] = modules[i].build();
            }
            var tmp = new SwerveDrive(this.initialPose, this.imu.get().build(), new_modules);
            this.apply(tmp);
            return tmp;
        }

        public void apply(SwerveDrive swerveDrive) {
            this.moduleDrivePID.ifPresent((PIDConstants constants) -> swerveDrive.setModuleDrivePID(constants));
            this.modulePivotPID.ifPresent((PIDConstants constants) -> swerveDrive.setModulePivotPID(constants));
            this.headingPID.ifPresent((PIDConstants constants) -> swerveDrive.setHeadingPID(constants));
            this.maxDriveSpeed.ifPresent((double speed) -> swerveDrive.setMaxDriveSpeed(speed, MetersPerSecond));
            this.maxTurnSpeed.ifPresent((double speed) -> swerveDrive.setMaxTurnSpeed(speed, RadiansPerSecond));
            this.maxDriveAccel.ifPresent((double accel) -> swerveDrive.setMaxDriveAccel(accel, MetersPerSecondPerSecond));
            this.maxTurnAccel.ifPresent((double accel) -> swerveDrive.setMaxTurnAccel(accel, RadiansPerSecond.per(Second)));
        }
    }
}
