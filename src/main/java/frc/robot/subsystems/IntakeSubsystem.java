package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorLevel;

public class IntakeSubsystem extends SubsystemBase {
    private ElevatorSubsystem elevatorSubsystem;
    private ClawSubsystem clawSubsystem;

    public IntakeSubsystem() {
        this.elevatorSubsystem = new ElevatorSubsystem();
        this.clawSubsystem = new ClawSubsystem();
    }

    public void score(ElevatorLevel height) {
        
    }
}
