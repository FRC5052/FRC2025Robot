package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    // Gearbox has ~8.53 gear ratio
    // Climb ~120:1 gear ratio
    // Idk why I'm putting that here lol
    private SparkMax elevatorMotor;
    private SparkMax followerMotor;
    private SparkMaxConfig config;
    private ElevatorLevel elevatorLevel;
    private SparkClosedLoopController elevatorPID;
    private ElevatorFeedforward feedforward;
    private DigitalInput bottomLimit;
    private boolean homed;

    public ElevatorSubsystem() {
        super();
        homed = true;

        elevatorMotor = new SparkMax(13, MotorType.kBrushless);
        followerMotor = new SparkMax(14, MotorType.kBrushless);

        bottomLimit = new DigitalInput(ElevatorConstants.kLimitSwitchPort);

        feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

        config = new SparkMaxConfig();
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, feedforward.calculate(0));
        config.closedLoop.maxMotion
            // TODO: multiply by gear ratio
            .maxAcceleration(3840.0)
            .maxVelocity(1920.0)
            .allowedClosedLoopError(0.5)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        config.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(elevatorMotor);

        followerMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        elevatorPID = elevatorMotor.getClosedLoopController();

        elevatorLevel = ElevatorLevel.Home;

        elevatorMotor.getEncoder().setPosition(0);
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

        public double level() {
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

    public void setLevel(ElevatorLevel height) {
        elevatorLevel = height;
    }

    public void resetLevel() {
        elevatorMotor.getEncoder().setPosition(0);
        elevatorLevel = ElevatorLevel.Home;
    }

    public ElevatorLevel getLevel() {
        return elevatorLevel;
    }

    private void setMotor(double position, AngleUnit unit) {
        elevatorPID.setReference(position, ControlType.kMAXMotionPositionControl);
    }

    public void rawSetMotor(double value) {
        elevatorMotor.set(value);
    }

    private void handleBounds() {
        if (bottomLimit.get()) {
            elevatorMotor.set(0);
            // elevatorMotor.getEncoder().setPosition(inchesToRotations(ElevatorConstants.bottom));
            elevatorMotor.getEncoder().setPosition(ElevatorConstants.bottom);
        }
        if (elevatorMotor.getEncoder().getPosition() > ElevatorConstants.top) {
        // if (elevatorMotor.getEncoder().getPosition() > inchesToRotations(ElevatorConstants.top)) {
            elevatorMotor.set(0);
            // elevatorMotor.getEncoder().setPosition(inchesToRotations(ElevatorConstants.top));
            elevatorMotor.getEncoder().setPosition(ElevatorConstants.top);
        }
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
        System.out.println(elevatorMotor.getEncoder().getPosition());
        config.closedLoop
            .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, feedforward.calculate(elevatorMotor.getEncoder().getVelocity()));

        elevatorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        if (homed) {
            setMotor(this.elevatorLevel.level(), Rotations);
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
}
