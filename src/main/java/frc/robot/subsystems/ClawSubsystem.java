package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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


    public ClawSubsystem() {
        super();
        this.pivotMotor = new SparkMax(ClawConstants.kPivotMotorID, MotorType.kBrushless);
        this.intakeMotor = new SparkMax(ClawConstants.kIntakeMotorID, MotorType.kBrushless);

        this.intakeLimit = new DigitalInput(ClawConstants.kLimitSwitchPort);

        this.pid = new ProfiledPIDController(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD, new TrapezoidProfile.Constraints(ClawConstants.kMaxVelocity, ClawConstants.kMaxAcceleration));
        
        this.clawPosition = ClawPosition.Idle;

        SmartDashboard.putData("elevator", this);
        SmartDashboard.putData("elevator/feedback", this.pid);
    }

    public enum ClawPosition {
        Idle(0.0),
        Score(1.0);

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
                case Score:
                    return Idle;
                default:
                    return null;
            }
        }
    }

    public void setPositionSetpoint(ClawPosition position) {
        this.clawPosition = position;
        pid.setGoal(position.position());
    }

    public void setIntakeVelocity(double velocity) {
        this.intakeMotor.set(velocity);
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

    public double getPositionSetpoint() {
        return pid.getGoal().position;
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

    public void setMotor() {
        double output = pid.calculate(getMeasuredPosition());
        pivotMotor.setVoltage(MathUtil.clamp(output, 0.0, pivotMotor.getBusVoltage()));
    }

    @Override
    public void periodic() {
        setMotor();
        // If limit switch activated, make sure velocity can only be set to negative (ejecting coral)
        // TODO Remove the short circuit (|| true)
        // if(!intakeLimit.get() || true) {
        //     setIntakeVelocity(Math.min(intakeMotor.getEncoder().getVelocity(), 0));
        // }
    }
}
