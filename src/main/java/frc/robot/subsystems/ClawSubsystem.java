package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private ProfiledPIDController pid;
    private ClawPosition clawPosition;
    private DigitalInput intakeLimit;
    private boolean setpointMode;


    public ClawSubsystem() {
        super();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        this.pivotMotor = new SparkMax(ClawConstants.kPivotMotorID, MotorType.kBrushless);
        pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.intakeMotor = new SparkMax(ClawConstants.kIntakeMotorID, MotorType.kBrushless);
        intakeMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


        this.intakeLimit = new DigitalInput(ClawConstants.kLimitSwitchPort);

        this.pid = new ProfiledPIDController(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD, new TrapezoidProfile.Constraints(ClawConstants.kMaxVelocity, ClawConstants.kMaxAcceleration));
        this.pid.setTolerance(0.01);
        this.clawPosition = ClawPosition.Idle;
        this.pivotMotor.getEncoder().setPosition(0);

        this.setpointMode = true;

        SmartDashboard.putData("elevator", this);
        SmartDashboard.putData("elevator/feedback", this.pid);
    }

    public enum ClawPosition {
        Idle(ClawConstants.kIdlePosition),
        Intake(ClawConstants.kIntakePosition),
        Score(ClawConstants.kScorePosition);

        private double level;

        private ClawPosition(double level) {
            this.level = level;
        }

        public double position() {
            return this.level;
        }

        public ClawPosition next() {
            switch (this) {
                case Idle:
                    return Score;
                case Intake:
                    return Score;
                case Score:
                    return Score;
                default:
                    return null;
            }
        }

        public ClawPosition prev() {
            switch (this) {
                case Idle:
                    return Idle;
                case Intake:
                    return Idle;
                case Score:
                    return Idle;
                default:
                    return null;
            }
        }
    }

    public void setPositionSetpoint(ClawPosition position) {
        this.clawPosition = position;
        setPositionSetpoint(position.position());
    }
    
    public void setPositionSetpoint(double position) {
        pid.setGoal(position);
    }

    private void setIntakeVelocity(double velocity) {
        this.intakeMotor.set(velocity);
    }

    public double getIntakeVelocity() {
        return this.intakeMotor.get();
    }

    public void scoreCoral() {
        // TODO Remove the short circuit (|| true)
        if(!intakeLimit.get() || true) {
            setIntakeVelocity(ClawConstants.kIntakeVelocity);
        }
    }

    public void intakeCoral() {
        // TODO Remove the short circuit (|| true)
        if(intakeLimit.get() || true) {
            setIntakeVelocity(-ClawConstants.kIntakeVelocity);
        }
    }

    public void resetIntake() {
        setIntakeVelocity(0);
    }

    public boolean isAtSetPoint() {
        return this.pid.atSetpoint();
    }

    public double getPositionSetpoint() {
        return pid.getGoal().position;
    }

    public ClawPosition getClawPosition() {
        return clawPosition;
    }

    public double getVelocitySetpoint() {
        return pid.getGoal().velocity;
    }

    public double getMeasuredPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    public double getMeasuredVelocity() {
        return pivotMotor.getEncoder().getVelocity();
    }

    public boolean getSetpointMode() {
        return setpointMode;
    }

    public void setSetpointMode(boolean b) {
        this.setpointMode = b;
    }

    public void setMotorSetpoint() {
        double output = pid.calculate(getMeasuredPosition());
        // double output = pid.getSetpoint().position-getMeasuredPosition()*0.5;
        // System.out.println("output: " + output + " | pos: " + getMeasuredPosition() + " | setpoint: " + getPositionSetpoint() + " | error: " + Math.abs(getMeasuredPosition()-getPositionSetpoint()));
        if (setpointMode) {
            pivotMotor.set(output);
        }
    }

    public void zeroClaw() {
        pivotMotor.getEncoder().setPosition(ClawConstants.kIdlePosition);
        clawPosition = ClawPosition.Idle;
        setpointMode = false;
    }

    public void setPivotVelocity(double speed) {
        if (!setpointMode) {
            pivotMotor.set(speed);
        }
    }

    // public void setMotor(double speed) {
    //     System.out.println(getMeasuredPosition());
    //     if (!(speed > 0 && getMeasuredPosition() > 0) && !(speed < 0 && getMeasuredPosition() < ClawConstants.kScorePosition)) {
    //         System.out.println("Moving claw " + speed);
    //         pivotMotor.set(speed);
    //     } else {
    //         System.out.println("Not moving claw");
    //         pivotMotor.stopMotor();
    //     }
    // }

    public void zeroEncoder() {
        pivotMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        // System.out.println(getMeasuredPosition());
        if (setpointMode) {
            setMotorSetpoint();
        }
        // setMotor();
        // If limit switch activated, make sure velocity can only be set to negative (ejecting coral)
        // TODO Remove the short circuit (|| true)
        // if(!intakeLimit.get() || true) {
        //     setIntakeVelocity(Math.min(intakeMotor.getEncoder().getVelocity(), 0));
        // }
    }
}
