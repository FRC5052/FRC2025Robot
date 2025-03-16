package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.ClawConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;
    private ProfiledPIDController pid;
    private AlgaeIntakePosition algaeIntakePosition;
    private DigitalInput intakeLimit;


    public AlgaeIntakeSubsystem() {
        this.pivotMotor = new SparkMax(AlgaeIntakeConstants.kPivotMotorID, MotorType.kBrushless);
        // config = new SparkMaxConfig();
        // config.inverted(true);
        // pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.intakeMotor = new SparkMax(AlgaeIntakeConstants.kIntakeMotorID, MotorType.kBrushless);

        this.intakeLimit = new DigitalInput(AlgaeIntakeConstants.kLimitSwitchPort);

        this.pid = new ProfiledPIDController(AlgaeIntakeConstants.kP, AlgaeIntakeConstants.kI, AlgaeIntakeConstants.kD, new TrapezoidProfile.Constraints(AlgaeIntakeConstants.kMaxVelocity, AlgaeIntakeConstants.kMaxAcceleration));
        this.pid.setTolerance(0.01);
        this.algaeIntakePosition = AlgaeIntakePosition.Score;
        this.pivotMotor.getEncoder().setPosition(0);

        SmartDashboard.putData("elevator", this);
        SmartDashboard.putData("elevator/feedback", this.pid);
    }

    public enum AlgaeIntakePosition {
        Idle(AlgaeIntakeConstants.kIdlePosition),
        Score(AlgaeIntakeConstants.kScorePosition);

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
        if(!intakeLimit.get() || true) {
            setIntakeVelocity(-AlgaeIntakeConstants.kIntakeVelocity);
        }
    }

    public void intakeAlgae() {
        if(intakeLimit.get() || true) {
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
        pivotMotor.set(MathUtil.clamp(output, -0.5, 0.5));
    }

    public void setPivotVelocity(double speed) {
        if (!(speed < 0 && getMeasuredPosition() < 0) && !(speed > 0 && getMeasuredPosition() > AlgaeIntakeConstants.kScorePosition)) {
            pivotMotor.set(speed);
        } else {
            pivotMotor.stopMotor();
        }
    }

    @Override
    public void periodic() {
        // System.out.println("pos: " + getMeasuredPosition() + " | setpoint: " + getPositionSetpoint() + " | error: " + Math.abs(getMeasuredPosition()-getPositionSetpoint()));
        // setMotor();
        // If limit switch activated, make sure velocity can only be set to negative (ejecting coral)
        // if(!intakeLimit.get()) {
        //     setIntakeVelocity(Math.min(intakeMotor.getEncoder().getVelocity(), 0));
        // }
    }
}
