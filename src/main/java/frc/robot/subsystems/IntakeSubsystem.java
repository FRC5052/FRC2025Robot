package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorLevel;

public class IntakeSubsystem extends SubsystemBase {
    public final ElevatorSubsystem elevatorSubsystem;
    public final ClawSubsystem clawSubsystem;

    public IntakeSubsystem() {
        this.elevatorSubsystem = new ElevatorSubsystem();
        this.clawSubsystem = new ClawSubsystem();
    }

    @Override
    public void periodic() {
        elevatorSubsystem.periodic();
    }
}
