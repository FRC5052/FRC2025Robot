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
    private SparkMax intakeMotor;
    private DigitalInput intakeLimit;


    public ClawSubsystem() {
        super();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        this.intakeMotor = new SparkMax(ClawConstants.kIntakeMotorID, MotorType.kBrushless);
        intakeMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        this.intakeLimit = new DigitalInput(ClawConstants.kLimitSwitchPort);

        SmartDashboard.putData("elevator", this);
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
}
