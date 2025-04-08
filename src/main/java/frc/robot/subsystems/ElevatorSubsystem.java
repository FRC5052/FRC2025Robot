package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import java.util.Arrays;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase implements Sendable {
    // Gearbox has ~8.53 gear ratio
    // Climb ~120:1 gear ratio
    // Idk why I'm putting that here lol
    private SparkMax elevatorMotor;
    private SparkMax followerMotor;
    private SparkMaxConfig config;
    private Optional<ElevatorLevel> elevatorLevel;
    private ElevatorFeedforward feedforward;
    private ProfiledPIDController feedback;
    private DigitalInput bottomLimit;
    private boolean homed;

    public ElevatorSubsystem() {
        super();
        homed = true;

        elevatorMotor = new SparkMax(ElevatorConstants.kElevatorMotorID, MotorType.kBrushless);
        followerMotor = new SparkMax(ElevatorConstants.kFollowerMotorID, MotorType.kBrushless);

        bottomLimit = new DigitalInput(ElevatorConstants.kLimitSwitchPort);

        feedback = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
        
        feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA, Robot.kDefaultPeriod);

        config = new SparkMaxConfig();
        config.encoder
            // .positionConversionFactor(Constants.ElevatorConstants.gearRatio)
            // .velocityConversionFactor(Constants.ElevatorConstants.gearRatio);
            // .positionConversionFactor(1.0 / Constants.ElevatorConstants.top)
            // .velocityConversionFactor(1.0);
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0);
        config.idleMode(IdleMode.kBrake);
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.idleMode(IdleMode.kBrake);
        followConfig.follow(elevatorMotor);

        followerMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        elevatorLevel = Optional.of(ElevatorLevel.Home);

        elevatorMotor.getEncoder().setPosition(0);
        SmartDashboard.putData("elevator", this);
        SmartDashboard.putData("elevator/feedback", this.feedback);
    }

    public enum ElevatorLevel {
        Home(-0),
        Intake(-10.25), // Original: -10.25
        L2(-27.71), // Original: -27.71
        L3(-30.09); // Original: -47.09

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
                    return Intake;
                case Intake:
                    return L2;
                case L2:
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
                case Intake:
                    return Home;
                case L2:
                    return Intake;
                case L3:
                    return L2;
                default:
                    return null;
            }
        }
    }

    public void setLevelSetpoint(ElevatorLevel level) {
        elevatorLevel = Optional.of(level);
        feedback.setGoal(level.height());
    }

    public void setHeightSetpoint(double height) {
        elevatorLevel = Optional.empty();
        feedback.setGoal(MathUtil.clamp(height, Constants.ElevatorConstants.top, 0.0));
    }

    public void resetLevel() {
        elevatorMotor.getEncoder().setPosition(0);
        feedback.setGoal(0.0);
        elevatorLevel = elevatorLevel.map((ElevatorLevel level) -> {
            return ElevatorLevel.Home;
        });
    }

    public Optional<ElevatorLevel> getLevelSetpoint() {
        return elevatorLevel;
    }

    public ElevatorLevel getLevelSetpointOrRound() {
        var values = ElevatorLevel.values();
        double setpoint = getHeightSetpoint();
        for (int i = 1; i < values.length; i++) {
            if (values[i-1].level >= setpoint && values[i].level <= setpoint) {
                double prevDist = Math.abs(values[i-1].level - setpoint);
                double nextDist = Math.abs(values[i].level - setpoint);

                if (prevDist < nextDist) {
                    return values[i-1];
                } else {
                    return values[i];
                }
            }
        }
        if (setpoint < values[values.length-1].level) return values[values.length-1];
        else if (setpoint > values[0].level) return values[0];
        return null;
    }

    public double getHeightSetpoint() {
        return feedback.getGoal().position;
    }

    public double getVelocitySetpoint() {
        return feedback.getGoal().velocity;
    }

    public double getMeasuredHeight() {
        return elevatorMotor.getEncoder().getPosition();
    }

    public double getMeasuredVelocity() {
        return elevatorMotor.getEncoder().getVelocity();
    }

    private void setMotor() {
        double output = feedback.calculate(getMeasuredHeight());
        output += feedforward.calculate(feedback.getSetpoint().velocity);
        elevatorMotor.setVoltage(MathUtil.clamp(output, -elevatorMotor.getBusVoltage(), elevatorMotor.getBusVoltage()));
    }

    // For use only for tuning feedforward, remove later

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
        // System.out.println(getMeasuredHeight());
        // System.out.println(feedforward.getKg() + " " + feedback.getP());
        if(DriverStation.isDisabled()){
            setLevelSetpoint(ElevatorLevel.Home);
        }

        if (homed) {
            setMotor();
        } else {
            homeElevator();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("feedforward/s", () -> this.feedforward.getKs(), (double value) -> {
            this.feedforward = new ElevatorFeedforward(
                value, 
                this.feedforward.getKg(), 
                this.feedforward.getKv(), 
                this.feedforward.getKa(), 
                this.feedforward.getDt()
            );
        });
        builder.addDoubleProperty("feedforward/g", () -> this.feedforward.getKg(), (double value) -> {
            System.out.println("Set g");
            this.feedforward = new ElevatorFeedforward(
                this.feedforward.getKs(), 
                value, 
                this.feedforward.getKv(), 
                this.feedforward.getKa(), 
                this.feedforward.getDt()
            );
        });
        builder.addDoubleProperty("feedforward/v", () -> this.feedforward.getKv(), (double value) -> {
            this.feedforward = new ElevatorFeedforward(
                this.feedforward.getKs(), 
                this.feedforward.getKg(), 
                value, 
                this.feedforward.getKa(), 
                this.feedforward.getDt()
            );
        });
        builder.addDoubleProperty("feedforward/a", () -> this.feedforward.getKa(), (double value) -> {
            this.feedforward = new ElevatorFeedforward(
                this.feedforward.getKs(), 
                this.feedforward.getKg(), 
                this.feedforward.getKv(), 
                value, 
                this.feedforward.getDt()
            );
        });
        builder.addDoubleProperty("measuredHeight", () -> this.getMeasuredHeight(), null);
        builder.addDoubleProperty("heightSetpoint", () -> this.getHeightSetpoint(), null);
        builder.addDoubleProperty("measuredVelocity", () -> this.getMeasuredVelocity(), null);
        builder.addDoubleProperty("velocitySetpoint", () -> this.getVelocitySetpoint(), null);
        builder.addDoubleArrayProperty("positionTuning", () -> new double[] {
            this.getMeasuredHeight(),
            this.getHeightSetpoint()
        }, null);
        builder.addDoubleArrayProperty("velocityTuning", () -> new double[] {
            this.getMeasuredVelocity(),
            this.getVelocitySetpoint()
        }, null);
    }
}
