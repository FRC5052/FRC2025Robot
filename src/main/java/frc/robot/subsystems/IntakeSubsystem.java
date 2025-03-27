package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.subsystems.ClawSubsystem.ClawPosition;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorLevel;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;
    private SparkMax followerMotor;
    private DigitalInput intakeLimit;

    public IntakeSubsystem() {
        super();
        this.intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        this.followerMotor = new SparkMax(IntakeConstants.kFollowerMotorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        
        intakeMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(intakeMotor, true);

        followerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        this.intakeLimit = new DigitalInput(IntakeConstants.kLimitSwitchPort);

        SmartDashboard.putData("intake", this);
    }

    private void setIntakeVelocity(double velocity) {
        this.intakeMotor.set(velocity);
    }

    public double getIntakeVelocity() {
        return this.intakeMotor.get();
    }

    public void intakeCoral() {
        // TODO Remove the short circuit (|| true)
        if(!intakeLimit.get() || true) {
            setIntakeVelocity(IntakeConstants.kIntakeVelocity);
        }
    }

    public void ejectCoral() {
        // TODO Remove the short circuit (|| true)
        if(intakeLimit.get() || true) {
            setIntakeVelocity(-IntakeConstants.kIntakeVelocity);
        }
    }

    public void resetIntake() {
        intakeMotor.stopMotor();
    }
}
