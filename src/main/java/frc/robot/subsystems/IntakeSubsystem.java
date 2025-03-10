package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.ClawSubsystem.ClawPosition;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorLevel;

// For readability, and to remove unnecessary overhead, the subsystems in this class have been moved into RobotContainer. - Andrew
@Deprecated
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

    // public void setClawPosition(ClawPosition position) {
    //     clawSubsystem.setPositionSetpoint(position);
    // }

    // public void intakeCoral() {
    //     clawSubsystem.intakeCoral();
    // }

    // public Command score(ElevatorLevel level, ClawPosition position) {
    //     Command scoreCommand = new Command() {
    //         @Override
    //         public void initialize() {
    //             setLevel(level);
    //             setClawPosition(position);
    //         }

    //         @Override
    //         public void execute() {
    //             if (Math.abs(elevatorSubsystem.getMeasuredHeight() - level.height()) < 0.1 && Math.abs(clawSubsystem.getMeasuredPosition() - position.position()) < 0.1) {
    //                 // Set claw subsystem to score
    //                 clawSubsystem.scoreCoral();
    //             }
    //         }

    //         @Override
    //         public void end(boolean interrupted) {
    //             clawSubsystem.resetIntake();
    //         }
    //     };

    //     return scoreCommand;
    // }

    @Override
    public void periodic() {
        elevatorSubsystem.periodic();
    }
}
