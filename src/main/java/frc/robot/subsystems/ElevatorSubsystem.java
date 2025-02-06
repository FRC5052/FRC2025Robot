package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import java.util.Optional;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    // Gearbox has ~8.53 gear ratio
    // Climb ~120:1 gear ratio
    // Idk why I'm putting that here lol
    private SparkMax elevatorMotor;
    private SparkMax followerMotor;
    private SparkMaxConfig config;
    private Optional<ElevatorLevel> elevatorLevel;
    private SparkClosedLoopController elevatorPID;
    private ElevatorFeedforward feedforward;
    private TrapezoidProfile.Constraints profileConstraints;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State profileSetpoint;
    private TrapezoidProfile.State profileGoal;
    private DigitalInput bottomLimit;
    private boolean homed;

    public ElevatorSubsystem() {
        super();
        homed = true;

        elevatorMotor = new SparkMax(13, MotorType.kBrushless);
        followerMotor = new SparkMax(14, MotorType.kBrushless);

        bottomLimit = new DigitalInput(ElevatorConstants.kLimitSwitchPort);

        profileConstraints = new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration);
        profile = new TrapezoidProfile(profileConstraints);
        profileSetpoint = new TrapezoidProfile.State();
        profileGoal = new TrapezoidProfile.State();
        
        feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

        config = new SparkMaxConfig();
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, feedforward.calculate(0));
        config.closedLoop.maxMotion
            // TODO: multiply by gear ratio
            .maxAcceleration(ElevatorConstants.kMaxAcceleration)
            .maxVelocity(ElevatorConstants.kMaxVelocity)
            .allowedClosedLoopError(0.5)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        config.encoder
            // .positionConversionFactor(Constants.ElevatorConstants.gearRatio)
            // .velocityConversionFactor(Constants.ElevatorConstants.gearRatio);
            // .positionConversionFactor(1.0 / Constants.ElevatorConstants.top)
            // .velocityConversionFactor(1.0);
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(elevatorMotor);

        followerMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        elevatorPID = elevatorMotor.getClosedLoopController();

        elevatorLevel = Optional.of(ElevatorLevel.Home);

        elevatorMotor.getEncoder().setPosition(0);
        SmartDashboard.putData("elevator", this);
    }

    public enum ElevatorLevel {
        Home(0.0),
        L0(10.0),
        L1(20.0),
        L2(30.0),
        Intake(40.0),
        L3(50.0);

        private double level;

        private ElevatorLevel(double level) {
            this.level = level;
        }

        public double height() {
            return this.level;
        }

        public ElevatorLevel next() {
            switch (this) {
                case Home:
                    return L0;
                case L0:
                    return L1;
                case L1:
                    return L2;
                case L2:
                    return Intake;
                case Intake:
                    return L3;
                case L3:
                    return L3;
                default:
                    return null;
            }
        }

        public ElevatorLevel prev() {
            switch (this) {
                case Home:
                    return Home;
                case L0:
                    return Home;
                case L1:
                    return L0;
                case L2:
                    return L1;
                case Intake:
                    return L2;
                case L3:
                    return Intake;
                default:
                    return null;
            }
        }
    }

    public void setLevelSetpoint(ElevatorLevel level) {
        elevatorLevel = Optional.of(level);
        profileGoal = new TrapezoidProfile.State(level.height(), 0);
    }

    public void setHeightSetpoint(double height) {
        elevatorLevel = Optional.empty();
        profileGoal = new TrapezoidProfile.State(MathUtil.clamp(height, 0.0, Constants.ElevatorConstants.top), 0);
    }

    public void resetLevel() {
        elevatorMotor.getEncoder().setPosition(0);
        profileGoal = new TrapezoidProfile.State(0, 0);
        elevatorLevel = elevatorLevel.map((ElevatorLevel level) -> {
            return ElevatorLevel.Home;
        });
    }

    public Optional<ElevatorLevel> getLevelSetpoint() {
        return elevatorLevel;
    }

    public double getHeightSetpoint() {
        return profileGoal.position;
    }

    public double getMeasuredHeight() {
        return elevatorMotor.getEncoder().getPosition();
    }

    private void setMotor(double position, AngleUnit unit) {
        elevatorPID.setReference(position, ControlType.kMAXMotionPositionControl);
    }

    // For use only for tuning feedforward, remove later
    public void setElevatorMotorVoltage(double voltage) {
        elevatorMotor.setVoltage(voltage);
    }

    // private double inchesToRotations(double inches) {
    //     //Find conversion from inches of elevator movement to rotations of motor
    // }

    // private double rotationsToInches(double rotations) {
    //     //Find conversion from rotations of motor to inches of elevator movement
    // }

    public void homeElevator() {
        elevatorMotor.set(-0.1); // Slow downward movement until bottom limit is hit
        if (!bottomLimit.get()) {
            elevatorMotor.stopMotor();
            resetLevel();
            homed = true;
        }
    }

    @Override
    public void periodic() {
        profileSetpoint = profile.calculate(Robot.kDefaultPeriod, profileSetpoint, profileGoal);
        config.closedLoop
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, feedforward.calculate(profileSetpoint.velocity));

        elevatorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if(!bottomLimit.get()){
            resetLevel();
        }

        if (homed) {
            setMotor(profileSetpoint.position, Rotations);
        } else {
            homeElevator();
        }
    }

    private void updateTelemetry() {
        // SmartDashboard.putNumber("Elevator Height", rotationsToInches(elevatorMotor.getEncoder().getPosition()));
        SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", elevatorMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Elevator Homed?", homed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("measuredHeight", () -> this.getMeasuredHeight(), null);
        builder.addDoubleProperty("heightSetpoint", () -> this.getHeightSetpoint(), null);
        builder.addDoubleArrayProperty("tuning", () -> new double[] {
            this.getMeasuredHeight(),
            this.getHeightSetpoint()
        }, null);
    }
}
