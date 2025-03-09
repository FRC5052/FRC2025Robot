// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.AddressableLEDSubsystem.AddressableLEDSlice;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  public final ClimbSubsystem m_climbSubsystem;
  public final AddressableLEDSubsystem m_ledSubsystem;
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

    this.m_swerveDriveSubsystem = new SwerveDriveSubsystem(
      () -> this.m_driverController.getRawAxis(1), 
      () -> this.m_driverController.getRawAxis(0), 
      () -> -this.m_driverController.getRawAxis(2)
      );
      
    this.m_elevatorSubsystem = new ElevatorSubsystem();
    this.m_clawSubsystem = new ClawSubsystem();

    this.m_climbSubsystem = new ClimbSubsystem();

    this.m_climbSubsystem.register();

    this.autoChooser = AutoBuilder.buildAutoChooser();
    

    // Shuffleboard.getTab("Driver Panel").add("Intake Camera", CameraServer.startAutomaticCapture()).withSize(6, 5).withPosition(0, 0);
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
    m_driverController.button(3).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.resetHeading()));
    m_driverController.button(4).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.setTargetPose(new Pose2d(9.5, 2.0, new Rotation2d(Radians.convertFrom(90.0, Degrees))))));

    Command secondControllerCommand = new Command() {

      @Override
      public void execute() {
        if (DriverStation.isAutonomousEnabled()) return;
      }
    };

    secondControllerCommand.setName("Teleop Intake");

    m_secondaryController.a().whileTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(m_elevatorSubsystem.getLevelSetpoint().get().next())));

    m_secondaryController.b().whileTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(m_elevatorSubsystem.getLevelSetpoint().get().prev())));

    m_secondaryController.y().onTrue(new InstantCommand(() -> m_elevatorSubsystem.resetLevel()));

    m_secondaryController.leftTrigger().onTrue(new InstantCommand(() -> {
      Optional<Pose2d> targetPose = Limelight.getScoringPose(
        m_swerveDriveSubsystem.getSwerveDrive().getPose(), 
        OperatorConstants.kScoreOffset,
        Meter
      );
      targetPose.ifPresent((Pose2d pose) -> m_swerveDriveSubsystem.setTargetPose(pose));
    }));

    // m_secondaryController.povUp().whileTrue(new Command() {
    //   @Override
    //   public void initialize() {
    //     m_clawSubsystem.scoreCoral();
    //   }

    //   @Override
    //   public void end(boolean interrupted) {
    //     m_clawSubsystem.resetIntake();
    //   }
    // });

    // m_secondaryController.povDown().whileTrue(new Command() {
    //   @Override
    //   public void initialize() {
    //     m_clawSubsystem.intakeCoral();
    //   }

    //   @Override
    //   public void end(boolean interrupted) {
    //     m_clawSubsystem.resetIntake();
    //   }
    // });

    // m_secondaryController.x().whileTrue(new Command() {
    //   private ClimbPosition position = ClimbPosition.Idle;

    //   @Override
    //   public void initialize() {
    //     position = position.next();
    //     m_climbSubsystem.setPositionSetpoint(position);
    //   }
    // });

    m_driverController.button(7).whileTrue(new Command() {
      private boolean on = true;

      @Override
      public void initialize() {
        on = !on;
        m_swerveDriveSubsystem.setFieldCentric(on);
      }
    });

    m_driverController.button(8).whileTrue(new Command() {
      private boolean on = true;

      @Override
      public void initialize() {
        on = !on;
        m_swerveDriveSubsystem.setFullSpeed(on);
      }
    });
    

    // LED Subsystem
    AddressableLEDSlice leftSlice = m_ledSubsystem.createSlice(LEDConstants.kLEDLeftSlice[0], LEDConstants.kLEDLeftSlice[1]-LEDConstants.kLEDLeftSlice[0]);
    AddressableLEDSlice topSlice = m_ledSubsystem.createSlice(LEDConstants.kLEDTopSlice[0], LEDConstants.kLEDTopSlice[1]-LEDConstants.kLEDTopSlice[0]);
    AddressableLEDSlice rightSlice = m_ledSubsystem.createSlice(LEDConstants.kLEDRightSlice[0], LEDConstants.kLEDRightSlice[1]-LEDConstants.kLEDRightSlice[0]);

    Command ledDefaultCommand = new Command() {
      private Timer timer = new Timer();

      @Override
      public void initialize() {
        timer.start();
      }

      @Override
      public void execute() {
        leftSlice.setRainbow(timer);
        topSlice.setRainbow(timer);
        rightSlice.setRainbow(timer);
        m_ledSubsystem.display();
      };
    };
    ledDefaultCommand.addRequirements(m_ledSubsystem);
    m_ledSubsystem.setDefaultCommand(ledDefaultCommand);
  }

  public void resetIntake() {

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
