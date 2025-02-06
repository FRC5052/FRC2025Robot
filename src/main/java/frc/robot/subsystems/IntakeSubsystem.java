package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorLevel;

public class IntakeSubsystem extends SubsystemBase {
    public final ElevatorSubsystem elevatorSubsystem;
    public final ClawSubsystem clawSubsystem;

    public IntakeSubsystem() {
        this.elevatorSubsystem = new ElevatorSubsystem();
        this.clawSubsystem = new ClawSubsystem();
    }

    public void setLevel(ElevatorLevel level) {
        elevatorSubsystem.setLevelSetpoint(level);
    }

    public void setLevel(double height) {
        elevatorSubsystem.setHeightSetpoint(height);
    }

    public Optional<ElevatorLevel> getLevel() {
        return elevatorSubsystem.getLevelSetpoint();
    }

    public Optional<ElevatorLevel> nextLevel() {
        return elevatorSubsystem.getLevelSetpoint().map((ElevatorLevel level) -> level.next());
    }

    public Optional<ElevatorLevel> prevLevel() {
        return elevatorSubsystem.getLevelSetpoint().map((ElevatorLevel level) -> level.prev());
    }

    @Override
    public void periodic() {
        elevatorSubsystem.periodic();
    }
}
