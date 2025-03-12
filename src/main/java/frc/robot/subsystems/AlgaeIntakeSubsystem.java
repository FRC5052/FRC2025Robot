package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private ProfiledPIDController pid;
    private AlgaeIntakePosition algaeIntakePosition;
    private DigitalInput intakeLimit;


    public AlgaeIntakeSubsystem() {
        this.pivotMotor = new SparkMax(AlgaeIntakeConstants.kPivotMotorID, MotorType.kBrushless);
        this.intakeMotor = new SparkMax(AlgaeIntakeConstants.kIntakeMotorID, MotorType.kBrushless);

        this.intakeLimit = new DigitalInput(AlgaeIntakeConstants.kLimitSwitchPort);

        this.pid = new ProfiledPIDController(AlgaeIntakeConstants.kP, AlgaeIntakeConstants.kI, AlgaeIntakeConstants.kD, new TrapezoidProfile.Constraints(AlgaeIntakeConstants.kMaxVelocity, AlgaeIntakeConstants.kMaxAcceleration));
        
        this.algaeIntakePosition = AlgaeIntakePosition.Score;

        SmartDashboard.putData("elevator", this);
        SmartDashboard.putData("elevator/feedback", this.pid);
    }

    public enum AlgaeIntakePosition {
        Idle(0.0),
        Score(1.0);

        private double level;

        private AlgaeIntakePosition(double level) {
            this.level = level;
        }

        public double position() {
            return this.level;
        }

        public AlgaeIntakePosition toggle() {
            switch (this) {
                case Idle:
                    return Score;
                case Score:
                    return Idle;
                default:
                    return null;
            }
        }

        public AlgaeIntakePosition next() {
            switch (this) {
                case Idle:
                    return Score;
                case Score:
                    return Score;
                default:
                    return null;
            }
        }

        public AlgaeIntakePosition prev() {
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

    public void setPositionSetpoint(AlgaeIntakePosition position) {
        this.algaeIntakePosition = position;
        pid.setGoal(position.position());
    }

    public void setIntakeVelocity(double velocity) {
        this.intakeMotor.set(velocity);
    }

    public void scoreAlgae() {
        if(!intakeLimit.get()) {
            setIntakeVelocity(-AlgaeIntakeConstants.kIntakeVelocity);
        }
    }

    public void intakeAlgae() {
        if(intakeLimit.get()) {
            setIntakeVelocity(AlgaeIntakeConstants.kIntakeVelocity);
        }
    }

    public void resetIntake() {
        setIntakeVelocity(0);
    }

    public double getPositionSetpoint() {
        return pid.getGoal().position;
    }

    public AlgaeIntakePosition getAlgaeIntakePosition() {
        return algaeIntakePosition;
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
        // System.out.println(getMeasuredPosition());
        setMotor();
        // If limit switch activated, make sure velocity can only be set to negative (ejecting coral)
        if(!intakeLimit.get()) {
            setIntakeVelocity(Math.min(intakeMotor.getEncoder().getVelocity(), 0));
        }
    }
}
