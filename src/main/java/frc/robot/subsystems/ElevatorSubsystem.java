package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;

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
    private ElevatorLevel elevatorLevel;
    private SparkClosedLoopController elevatorPID;
    private ElevatorFeedforward feedforward;
    private DigitalInput bottomLimit;

    public ElevatorSubsystem() {
        elevatorMotor = new SparkMax(0, MotorType.kBrushless);
        followerMotor = new SparkMax(0, MotorType.kBrushless);

        bottomLimit = new DigitalInput(ElevatorConstants.kLimitSwitchPort);

        feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0, 0, 0, feedforward.calculate(0, 0));
        config.closedLoop.maxMotion
            .maxAcceleration(MetersPerSecondPerSecond.of(0).magnitude())
            .maxVelocity(MetersPerSecond.of(0).magnitude())
            .allowedClosedLoopError(Meters.of(0.001).magnitude())
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(elevatorMotor);
        followerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorPID = elevatorMotor.getClosedLoopController();

        elevatorLevel = ElevatorLevel.L0;
    }

    public enum ElevatorLevel {
        L0,
        L1,
        L2,
        Intake,
        L3,
    }

    public void setLevel(ElevatorLevel height) {
        this.elevatorLevel = height;
    }

    public void setMotor(double position, AngleUnit unit) {
        elevatorPID.setReference(position, ControlType.kMAXMotionPositionControl);
    }

    private void handleBounds() {
        if (bottomLimit.get()) {
            elevatorMotor.set(0);
            elevatorMotor.getEncoder().setPosition(ElevatorConstants.bottom);
        }
        if (elevatorMotor.getEncoder().getPosition() > ElevatorConstants.top) {
            elevatorMotor.set(0);
            elevatorMotor.getEncoder().setPosition(ElevatorConstants.top);
        }
    }

    private double calculateFeedForward(double velocity, AngleUnit unit) {
        double vel = Rotations.convertFrom(velocity, unit);
        return (ElevatorConstants.kS * Math.signum(vel)) + ElevatorConstants.kG + (ElevatorConstants.kV * vel);
    }

    @Override
    public void periodic() {
        switch (this.elevatorLevel) {
            case L0:
                setMotor(0, Rotations); //TODO: Figure out corresponding angular position
                break;
            case L1:
                setMotor(0, Rotations); //TODO: Figure out corresponding angular position
                break;
            case L2:
                setMotor(0, Rotations); //TODO: Figure out corresponding angular position
                break;
            case Intake:
                setMotor(0, Rotations);
                break;
            case L3:
                setMotor(0, Rotations); //TODO: Figure out corresponding angular position
                break;
            default:
                setMotor(0, Rotations); //TODO: Figure out corresponding angular position
                break;
        }
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Height", );
        SmartDashboard.putNumber("Elevator Target", );
        SmartDashboard.putBoolean("Elevator Homed", );
        SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", );
    }
}
