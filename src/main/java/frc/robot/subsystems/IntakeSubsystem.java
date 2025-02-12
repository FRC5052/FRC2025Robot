package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
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

    public Command score(ElevatorLevel level) {
        Command scoreCommand = new Command() {
            @Override
            public void initialize() {
                setLevel(level);
            }

            @Override
            public void execute() {
                if (Math.abs(getLevel().orElse(ElevatorLevel.Home).height() - level.height()) < 0.1) {
                    // Set claw subsystem to score
                }
            }
        };

        return scoreCommand;
    }

    @Override
    public void periodic() {
        elevatorSubsystem.periodic();
    }
}
