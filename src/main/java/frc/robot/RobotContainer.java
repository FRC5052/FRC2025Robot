// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.AddressableLEDSubsystem;
// import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AddressableLEDSubsystem.AddressableLEDSlice;
// import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakePosition;
import frc.robot.subsystems.ClimbSubsystem.ClimbPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
// import frc.robot.subsystems.ClimbSubsystem.ClimbPosition;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorLevel;

import static edu.wpi.first.units.Units.*;

import java.lang.invoke.MethodHandles.Lookup.ClassOption;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.tree.DefaultTreeModel;
import javax.swing.tree.TreeNode;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.hal.REVPHJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.swerve.SwerveDrive.HeadingControlMode;
import frc.robot.swerve.SwerveEncoder.CANCoderSwerveEncoder;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveMotor;
import frc.robot.swerve.SwerveMotor.SparkMaxSwerveMotor;
import frc.robot.swerve.SwerveMotor.SwerveMotorType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public final SwerveDriveSubsystem m_swerveDriveSubsystem;
  public final ElevatorSubsystem m_elevatorSubsystem;
  public final ClawSubsystem m_clawSubsystem;
  public final IntakeSubsystem m_intakeSubsystem;
  // public final ClimbSubsystem m_climbSubsystem;
  // public final AlgaeIntakeSubsystem m_algaeIntakeSubsystem;
  public final AddressableLEDSubsystem m_ledSubsystem;
  private AddressableLEDSlice leftSlice;
  private AddressableLEDSlice topSlice;
  private AddressableLEDSlice rightSlice;
  // public final ColorSensorSubsystem m_colorSensorSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandGenericHID m_driverController =
      new CommandGenericHID(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_secondaryController =
      new CommandXboxController(1);

  private static RobotContainer instance;

  private SendableChooser<Command> autoChooser;

  public static RobotContainer getStaticInstance() {
    return instance;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    instance = this;

    m_ledSubsystem = new AddressableLEDSubsystem(LEDConstants.kLEDPort, LEDConstants.kLEDLength);
    leftSlice = m_ledSubsystem.createSlice(LEDConstants.kLEDLeftSlice[0], LEDConstants.kLEDLeftSlice[1]-LEDConstants.kLEDLeftSlice[0]);
    topSlice = m_ledSubsystem.createSlice(LEDConstants.kLEDTopSlice[0], LEDConstants.kLEDTopSlice[1]-LEDConstants.kLEDTopSlice[0]);
    rightSlice = m_ledSubsystem.createSlice(LEDConstants.kLEDRightSlice[0], LEDConstants.kLEDRightSlice[1]-LEDConstants.kLEDRightSlice[0], true);

    this.m_swerveDriveSubsystem = new SwerveDriveSubsystem(
      () -> this.m_driverController.getRawAxis(1), 
      () -> this.m_driverController.getRawAxis(0), 
      () -> -this.m_driverController.getRawAxis(2)
    );
    m_swerveDriveSubsystem.setFieldCentric(true);
      
    this.m_elevatorSubsystem = new ElevatorSubsystem();
    // DO NOT DO THIS
    // this.m_elevatorSubsystem.register();

    this.m_clawSubsystem = new ClawSubsystem();
    // DO NOT DO THIS
    // this.m_clawSubsystem.register();

    this.m_intakeSubsystem = new IntakeSubsystem();

    // this.m_algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    // this.m_algaeIntakeSubsystem.register();

    // this.m_climbSubsystem = new ClimbSubsystem();
    // DO NOT DO THIS
    // this.m_climbSubsystem.register();

    NamedCommands.registerCommand("Home", new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(ElevatorLevel.Home)));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(ElevatorLevel.Intake)));
    NamedCommands.registerCommand("L1", new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(ElevatorLevel.L1)));
    NamedCommands.registerCommand("L2", new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(ElevatorLevel.L2)));
    NamedCommands.registerCommand("L3", new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(ElevatorLevel.L3)));
    
    NamedCommands.registerCommand("ClawScore", new InstantCommand(() -> {
      m_clawSubsystem.scoreCoral();
    }));
    NamedCommands.registerCommand("ResetClaw", new InstantCommand(() -> {
      m_clawSubsystem.resetIntake();
    }));
    NamedCommands.registerCommand("IntakeCoral", new InstantCommand(() -> {
      m_intakeSubsystem.intakeCoral();
    }));
    NamedCommands.registerCommand("ResetCoralIntake", new InstantCommand(() -> {
      m_clawSubsystem.resetIntake();
    }));

    this.autoChooser = AutoBuilder.buildAutoChooser();
    

    Shuffleboard.getTab("Driver Panel").add("Intake Camera", CameraServer.startAutomaticCapture()).withSize(6, 5).withPosition(0, 0);
    Shuffleboard.getTab("Driver Panel").add("Auto Chooser", this.autoChooser).withSize(2, 1).withPosition(6, 0);
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // Reset Heading
    m_driverController.button(3).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.resetHeading()));

    // Auto-Align Left
    m_driverController.button(7).debounce(0.1).onTrue(
      new InstantCommand(() -> {
        Optional<Pose2d> targetPose = Limelight.getScoringPose(
            m_swerveDriveSubsystem.getSwerveDrive().getPose(), 
            OperatorConstants.kScoreLeftOffset,
            Meter
          );
          targetPose.ifPresent((Pose2d pose) -> {
            m_swerveDriveSubsystem.setTargetPose(pose);
            topSlice.setColor(0, 255, 0);
          });
      })
    );
    //Auto-Align Right
    m_driverController.button(8).debounce(0.1).onTrue(
      new InstantCommand(() -> {
        Optional<Pose2d> targetPose = Limelight.getScoringPose(
            m_swerveDriveSubsystem.getSwerveDrive().getPose(), 
            OperatorConstants.kScoreRightOffset,
            Meter
          );
          targetPose.ifPresent((Pose2d pose) -> {
            m_swerveDriveSubsystem.setTargetPose(pose);
            topSlice.setColor(0, 255, 0);
          });
      })
    );

    // Hold trigger for field centric
    m_driverController.button(1).whileTrue(new Command() {
      @Override
      public void execute() {
        m_swerveDriveSubsystem.setFieldCentric(false);
      }
      @Override
      public void end(boolean interrupted) {
        m_swerveDriveSubsystem.setFieldCentric(true);
      }
    });

    // Makes sure controller can't do anything during auton
    Command secondControllerCommand = new Command() {

      @Override
      public void execute() {
        if (DriverStation.isAutonomousEnabled()) return;
      }
    };
    secondControllerCommand.setName("Teleop Intake");

    // Increase Elevator Level
    m_secondaryController.rightTrigger().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(m_elevatorSubsystem.getLevelSetpoint().get().next())));

    // Decrease Elevator Level
    m_secondaryController.leftTrigger().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(m_elevatorSubsystem.getLevelSetpoint().get().prev())));

    // // When y pushed, switch to manual mode and move up
    // m_secondaryController.y().whileTrue(new StartEndCommand(
    //   () -> {
    //     m_clawSubsystem.setSetpointMode(false);
    //     m_clawSubsystem.setPivotVelocity(0.05);
    //   }, 
    //   () -> m_clawSubsystem.setPivotVelocity(0)
    // ));

    // // When a pushed, switch to manual mode and move down
    // m_secondaryController.a().whileTrue(new StartEndCommand(
    //   () -> {
    //     m_clawSubsystem.setSetpointMode(false);
    //     m_clawSubsystem.setPivotVelocity(-0.05);
    //   }, 
    //   () -> m_clawSubsystem.setPivotVelocity(0)
    // ));

    // // Toggle Claw Level
    // // If left joystick up, set claw to Idle position
    // m_secondaryController.axisGreaterThan(1, -0.5).onTrue(new InstantCommand(() -> {
    //   m_clawSubsystem.setSetpointMode(true);
    //   m_clawSubsystem.setPositionSetpoint(ClawPosition.Idle);
    // }));
    // // If left joystick down, set claw to Score position
    // m_secondaryController.axisGreaterThan(1, 0.5).onTrue(new InstantCommand(() -> {
    //   m_clawSubsystem.setSetpointMode(true);
    //   m_clawSubsystem.setPositionSetpoint(ClawPosition.Score);
    // }));

    // // If start clicked, set claw to Intake position
    // m_secondaryController.start().onTrue(new InstantCommand(() -> m_clawSubsystem.setPositionSetpoint(ClawPosition.Intake)));

    // // If select pressed, zero the claw
    // m_secondaryController.button(7).onTrue(new InstantCommand(() -> {
    //   m_clawSubsystem.zeroClaw();
    // }));

    // When x pressed, release coral, when released, stop intake
    m_secondaryController.x().whileTrue(new StartEndCommand(
      () -> {
        m_clawSubsystem.scoreCoral();
        m_intakeSubsystem.intakeCoral();
      }, 
      () -> {
        m_clawSubsystem.resetIntake();
        m_intakeSubsystem.resetIntake();
      }, 
      m_clawSubsystem
    ));

    // When b pressed, intake coral, when released, stop intake
    m_secondaryController.b().whileTrue(new StartEndCommand(
      () -> {
        m_clawSubsystem.intakeCoral();
        m_intakeSubsystem.ejectCoral();
      }, 
      () -> {
        m_clawSubsystem.resetIntake();
        m_intakeSubsystem.resetIntake();
      }, 
      m_clawSubsystem
    ));

    // Command scoreAlgaeCommand = new StartEndCommand(
    //   () -> m_algaeIntakeSubsystem.scoreAlgae(),
    //   () -> m_algaeIntakeSubsystem.resetIntake()  
    // );
    // Command intakeAlgaeCommand = new StartEndCommand(
    //   () -> m_algaeIntakeSubsystem.intakeAlgae(),
    //   () -> m_algaeIntakeSubsystem.resetIntake()  
    // );

    // // Algae Arm Out
    // m_secondaryController.povUp().whileTrue(new StartEndCommand(
    //   () -> m_algaeIntakeSubsystem.setPivotVelocity(0.05), 
    //   () -> m_algaeIntakeSubsystem.setPivotVelocity(0)
    // ));

    // // Algae Arm In
    // m_secondaryController.povDown().whileTrue(new StartEndCommand(
    //   () -> m_algaeIntakeSubsystem.setPivotVelocity(-0.05), 
    //   () -> m_algaeIntakeSubsystem.setPivotVelocity(0)
    // ));

    // // Score Algae
    // m_secondaryController.povRight().whileTrue(scoreAlgaeCommand);

    // // Intake Algae
    // m_secondaryController.povLeft().whileTrue(intakeAlgaeCommand);

    // // Move algae arm down and intake
    // m_secondaryController.povDownLeft().whileTrue(
    //   new StartEndCommand(
    //     () -> {
    //       // m_algaeIntakeSubsystem.setPositionSetpoint(AlgaeIntakePosition.Idle);
    //       m_algaeIntakeSubsystem.setPivotVelocity(-0.05);
    //       m_algaeIntakeSubsystem.intakeAlgae();
    //     },
    //     () -> {
    //       m_algaeIntakeSubsystem.resetIntake();
    //       m_algaeIntakeSubsystem.setPivotVelocity(0);
    //     }
    //   )
    // );

    // // Move algae arm up and score
    // m_secondaryController.povUpRight().whileTrue(
    //   new StartEndCommand(
    //     () -> {
    //       // m_algaeIntakeSubsystem.setPositionSetpoint(AlgaeIntakePosition.Score);
    //       m_algaeIntakeSubsystem.setPivotVelocity(0.05);
    //       m_algaeIntakeSubsystem.scoreAlgae();
    //     },
    //     () -> {
    //       m_algaeIntakeSubsystem.resetIntake();
    //       m_algaeIntakeSubsystem.setPivotVelocity(0);
    //     }
    //   )
    // );

    // Toggle field-centric
    // m_driverController.button(7).whileTrue(new Command() {
    //   private boolean on = true;

    //   @Override
    //   public void initialize() {
    //     on = !on;
    //     m_swerveDriveSubsystem.setFieldCentric(on);
    //   }
    // });

    // Toggle slow mode
    // m_driverController.button(8).whileTrue(new Command() {
    //   private boolean on = true;

    //   @Override
    //   public void initialize() {
    //     on = !on;
    //     m_swerveDriveSubsystem.setFullSpeed(on);
    //   }
    // });
    

    // LED Subsystem
    Command ledIdleCommand = new Command() {
      Timer timer = new Timer();
      double height;
      double setHeight;

      @Override
      public void initialize() {
          timer.start();
          height = m_elevatorSubsystem.getMeasuredHeight();
          setHeight = m_elevatorSubsystem.getHeightSetpoint();
      }

      @Override
      public void execute() {
        // double distToNearestScore = m_swerveDriveSubsystem.getSwerveDrive().getPosePositionMeters().getDistance(m_swerveDriveSubsystem.getSwerveDrive().getPose().nearest(null).getTranslation());
        if (Limelight.hasReefScoreTag()) {
          topSlice.setColor(0, 255, 0);
        } else {
          topSlice.setColor(0, 0, 0);
        }
        if (m_elevatorSubsystem.getHeightSetpoint() < 0.5) {
          leftSlice.setRainbow(timer);
          rightSlice.setRainbow(timer);
        } else {
          height = m_elevatorSubsystem.getMeasuredHeight();
          setHeight = m_elevatorSubsystem.getHeightSetpoint();
          leftSlice.fill(Color.kBlack);
          rightSlice.fill(Color.kBlack);
          if (height <= setHeight) {
            leftSlice.setMeter(ElevatorConstants.top, setHeight, Color.kOrange, null);
            leftSlice.setMeter(ElevatorConstants.top, height, Color.kRed, null);
            rightSlice.setMeter(ElevatorConstants.top, setHeight, Color.kOrange, null);
            rightSlice.setMeter(ElevatorConstants.top, height, Color.kRed, null);
          } else {
            leftSlice.setMeter(ElevatorConstants.top, height, Color.kOrange, null);
            leftSlice.setMeter(ElevatorConstants.top, setHeight, Color.kRed, null);
            rightSlice.setMeter(ElevatorConstants.top, height, Color.kOrange, null);
            rightSlice.setMeter(ElevatorConstants.top, setHeight, Color.kRed, null);
          }
        }
        m_ledSubsystem.display();
      };
    };
    ledIdleCommand.addRequirements(m_ledSubsystem);
    m_ledSubsystem.setDefaultCommand(ledIdleCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    var command = autoChooser.getSelected();
    // if (command != null) {
    //   command.addRequirements(m_intakeShooterSubsystem);
    //   command.addRequirements(m_swerveDriveSubsystem);
    // }
    return command;
  }
}
