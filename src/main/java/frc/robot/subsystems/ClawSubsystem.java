package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private SparkMax pivotMotor;
    private SparkMax intakeMotor;

    public ClawSubsystem() {
        // this.pivotMotor = new SparkMax(0, MotorType.kBrushless);
        // this.intakeMotor = new SparkMax(0, MotorType.kBrushless);
    }

    public void setIntakeVelocity(double velocity) {
        this.intakeMotor.set(velocity);
    }

    public void resetClaw() {
        
    }
}
