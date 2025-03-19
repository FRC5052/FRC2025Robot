// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AddressableLEDSubsystem.AddressableLEDSlice;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakePosition;
import frc.robot.subsystems.ClawSubsystem.ClawPosition;
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
  public final ClimbSubsystem m_climbSubsystem;
  public final AlgaeIntakeSubsystem m_algaeIntakeSubsystem;
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
    rightSlice = m_ledSubsystem.createSlice(LEDConstants.kLEDRightSlice[0], LEDConstants.kLEDRightSlice[1]-LEDConstants.kLEDRightSlice[0]);

    this.m_swerveDriveSubsystem = new SwerveDriveSubsystem(
      () -> this.m_driverController.getRawAxis(1), 
      () -> this.m_driverController.getRawAxis(0), 
      () -> -this.m_driverController.getRawAxis(2)
    );
    m_swerveDriveSubsystem.setFieldCentric(true);
      
    this.m_elevatorSubsystem = new ElevatorSubsystem();
    this.m_elevatorSubsystem.register();

    this.m_clawSubsystem = new ClawSubsystem();
    this.m_clawSubsystem.register();

    this.m_algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    this.m_algaeIntakeSubsystem.register();

    this.m_climbSubsystem = new ClimbSubsystem();
    this.m_climbSubsystem.register();

    NamedCommands.registerCommand("L2/Trough", new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(ElevatorLevel.L2)));
    NamedCommands.registerCommand("L3", new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(ElevatorLevel.L3)));
    NamedCommands.registerCommand("Coral", new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(ElevatorLevel.Coral)));
    NamedCommands.registerCommand("L4", new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(ElevatorLevel.L4)));
    
    NamedCommands.registerCommand("ScoreCoral", new InstantCommand(() -> m_clawSubsystem.scoreCoral()));
    NamedCommands.registerCommand("ResetCoralIntake", new InstantCommand(() -> m_clawSubsystem.resetIntake()));

    NamedCommands.registerCommand("ClawUp", new InstantCommand(() -> m_clawSubsystem.setPositionSetpoint(ClawPosition.Idle)));
    NamedCommands.registerCommand("ClawIntake", new InstantCommand(() -> m_clawSubsystem.setPositionSetpoint(ClawPosition.Intake)));
    NamedCommands.registerCommand("ClawDown", new InstantCommand(() -> m_clawSubsystem.setPositionSetpoint(ClawPosition.Score)));

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

    // Auto-Align
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

    // Toggle field centric
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
    
    // m_driverController.button(3).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.resetHeading()));
    // m_driverController.button(4).onTrue(new InstantCommand(() -> m_swerveDriveSubsystem.setTargetPose(new Pose2d(9.5, 2.0, new Rotation2d(Radians.convertFrom(90.0, Degrees))))));

    // Fine Adjustment
    m_driverController.povLeft().onTrue(
      new InstantCommand(() -> 
        m_swerveDriveSubsystem.setTargetPose(
          m_swerveDriveSubsystem.getSwerveDrive().getPose().plus(new Transform2d(0,-0.3,new Rotation2d(0)))
        )
      )
    );
    m_driverController.povRight().onTrue(
      new InstantCommand(() -> 
        m_swerveDriveSubsystem.setTargetPose(
          m_swerveDriveSubsystem.getSwerveDrive().getPose().plus(new Transform2d(0,0.3,new Rotation2d(0)))
        )
      )
    );
    m_driverController.povUp().onTrue(
      new InstantCommand(() -> 
        m_swerveDriveSubsystem.setTargetPose(
          m_swerveDriveSubsystem.getSwerveDrive().getPose().plus(new Transform2d(0.3,0,new Rotation2d(0)))
        )
      )
    );
    m_driverController.povDown().onTrue(
      new InstantCommand(() -> 
        m_swerveDriveSubsystem.setTargetPose(
          m_swerveDriveSubsystem.getSwerveDrive().getPose().plus(new Transform2d(-0.3,0,new Rotation2d(0)))
        )
      )
    );

    Command secondControllerCommand = new Command() {

      @Override
      public void execute() {
        if (DriverStation.isAutonomousEnabled()) return;
      }
    };

    secondControllerCommand.setName("Teleop Intake");

    Command ledElevatorHeightCommand = new Command() {
      double height;
      double setHeight;

      @Override
      public void initialize() {
        height = m_elevatorSubsystem.getMeasuredHeight();
        setHeight = m_elevatorSubsystem.getHeightSetpoint();
      }

      @Override
      public void execute() {
        if (height <= setHeight) {
          leftSlice.setMeter(ElevatorConstants.top, setHeight, Color.kOrange, Color.kBlack);
          leftSlice.setMeter(ElevatorConstants.top, height, Color.kRed, Color.kBlack);
          rightSlice.setMeter(ElevatorConstants.top, setHeight, Color.kOrange, Color.kBlack);
          rightSlice.setMeter(ElevatorConstants.top, height, Color.kRed, Color.kBlack);
        } else {
          leftSlice.setMeter(ElevatorConstants.top, height, Color.kOrange, Color.kBlack);
          leftSlice.setMeter(ElevatorConstants.top, setHeight, Color.kRed, null);
          rightSlice.setMeter(ElevatorConstants.top, height, Color.kOrange, Color.kBlack);
          rightSlice.setMeter(ElevatorConstants.top, setHeight, Color.kRed, null);
        }
        height = m_elevatorSubsystem.getMeasuredHeight();
        setHeight = m_elevatorSubsystem.getHeightSetpoint();
      }
    };

    
    Command scoreCoralCommand = new SequentialCommandGroup(
      new InstantCommand(() -> m_clawSubsystem.scoreCoral()),
      new WaitCommand(1.0),
      new InstantCommand(() -> m_clawSubsystem.resetIntake())
    );
    scoreCoralCommand.addRequirements(m_clawSubsystem);

    Command intakeCoralCommand = new SequentialCommandGroup(
      new InstantCommand(() -> m_clawSubsystem.intakeCoral()),
      new WaitCommand(1.0),
      new InstantCommand(() -> m_clawSubsystem.resetIntake())
    );
    intakeCoralCommand.addRequirements(m_clawSubsystem);

    

    InstantCommand resetClawCommand = new InstantCommand(() -> m_clawSubsystem.setPositionSetpoint(ClawPosition.Idle));
    resetClawCommand.addRequirements(m_clawSubsystem);

    // Increase Level
    m_secondaryController.rightTrigger().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(m_elevatorSubsystem.getLevelSetpoint().get().next())));

    // Decrease Level
    m_secondaryController.leftTrigger().onTrue(new InstantCommand(() -> m_elevatorSubsystem.setLevelSetpoint(m_elevatorSubsystem.getLevelSetpoint().get().prev())));

    // Toggle Claw Level
    // m_secondaryController.a().onTrue(new ConditionalCommand(
    //   new InstantCommand(() -> m_clawSubsystem.setPositionSetpoint(ClawPosition.Score)), 
    //   new InstantCommand(() -> m_clawSubsystem.setPositionSetpoint(ClawPosition.Idle)), 
    //   () -> m_clawSubsystem.getClawPosition() == ClawPosition.Idle
    // ));

    // // When y clicked, turn to velocity mode, then reset pivot encoder to 0 when released
    // m_secondaryController.y().whileTrue(new Command() {
    //   @Override
    //   public void execute() {
    //       if (m_secondaryController.getLeftY() < -0.5) {
    //         m_clawSubsystem.setPivotVelocity(0.05);
    //       } else if (m_secondaryController.getLeftY() > 0.5) {
    //         m_clawSubsystem.setPivotVelocity(-0.05);
    //       } else {
    //         m_clawSubsystem.setPivotVelocity(0);
    //       }
    //   }

    //   @Override
    //       public void end(boolean interrupted) {
    //           m_clawSubsystem.setPivotVelocity(0);
    //           m_clawSubsystem.zeroEncoder();
    //           m_clawSubsystem.setPositionSetpoint(ClawPosition.Idle);
    //       }
    // });

    m_secondaryController.y().whileTrue(new StartEndCommand(
      () -> {
        m_clawSubsystem.setSetpointMode(false);
        m_clawSubsystem.setPivotVelocity(0.05);
      }, 
      () -> m_clawSubsystem.setPivotVelocity(0)
    ));

    m_secondaryController.a().whileTrue(new StartEndCommand(
      () -> {
        m_clawSubsystem.setSetpointMode(false);
        m_clawSubsystem.setPivotVelocity(-0.05);
      }, 
      () -> m_clawSubsystem.setPivotVelocity(0)
    ));

    // Score Coral
    m_secondaryController.x().whileTrue(new StartEndCommand(
      () -> m_clawSubsystem.scoreCoral(), 
      () -> m_clawSubsystem.resetIntake(), 
      m_clawSubsystem
    ));

    // Intake Coral
    m_secondaryController.b().whileTrue(new StartEndCommand(
      () -> m_clawSubsystem.intakeCoral(), 
      () -> m_clawSubsystem.resetIntake(), 
      m_clawSubsystem
    ));

    // Toggle Claw Level
    // If left joystick up
    m_secondaryController.axisGreaterThan(1, -0.5).onTrue(new InstantCommand(() -> {
        m_clawSubsystem.setSetpointMode(true);
        m_clawSubsystem.setPositionSetpoint(ClawPosition.Idle);
    }));
    // If left joystick down
    m_secondaryController.axisGreaterThan(1, 0.5).onTrue(new InstantCommand(() -> {
      m_clawSubsystem.setSetpointMode(true);
      m_clawSubsystem.setPositionSetpoint(ClawPosition.Score);
  }));

    m_secondaryController.start().onTrue(new InstantCommand(() -> m_clawSubsystem.setPositionSetpoint(ClawPosition.Intake)));

    m_secondaryController.button(7).onTrue(new InstantCommand(() -> {
      m_clawSubsystem.zeroClaw();
    }));

    Command scoreAlgaeCommand = new StartEndCommand(
      () -> m_algaeIntakeSubsystem.scoreAlgae(),
      () -> m_algaeIntakeSubsystem.resetIntake()  
    );

    Command intakeAlgaeCommand = new StartEndCommand(
      () -> m_algaeIntakeSubsystem.intakeAlgae(),
      () -> m_algaeIntakeSubsystem.resetIntake()  
    );
    
    Command algaeArmOutCommand = new InstantCommand(() -> m_algaeIntakeSubsystem.setPositionSetpoint(AlgaeIntakePosition.Score));
    
    Command algaeArmInCommand = new InstantCommand(() -> m_algaeIntakeSubsystem.setPositionSetpoint(AlgaeIntakePosition.Idle));

    // Algae Arm Out
    m_secondaryController.povUp().whileTrue(new StartEndCommand(
      () -> m_algaeIntakeSubsystem.setPivotVelocity(0.05), 
      () -> m_algaeIntakeSubsystem.setPivotVelocity(0)
    ));

    // Algae Arm In
    m_secondaryController.povDown().whileTrue(new StartEndCommand(
      () -> m_algaeIntakeSubsystem.setPivotVelocity(-0.05), 
      () -> m_algaeIntakeSubsystem.setPivotVelocity(0)
    ));

    m_secondaryController.povRight().whileTrue(scoreAlgaeCommand);
    m_secondaryController.povLeft().whileTrue(intakeAlgaeCommand);

    m_secondaryController.povDownLeft().whileTrue(
      new StartEndCommand(
        () -> {
          // m_algaeIntakeSubsystem.setPositionSetpoint(AlgaeIntakePosition.Idle);
          m_algaeIntakeSubsystem.setPivotVelocity(-0.05);
          m_algaeIntakeSubsystem.intakeAlgae();
        },
        () -> {
          m_algaeIntakeSubsystem.resetIntake();
          m_algaeIntakeSubsystem.setPivotVelocity(0);
        }
      )
    );

    m_secondaryController.povUpRight().whileTrue(
      new StartEndCommand(
        () -> {
          // m_algaeIntakeSubsystem.setPositionSetpoint(AlgaeIntakePosition.Score);
          m_algaeIntakeSubsystem.setPivotVelocity(0.05);
          m_algaeIntakeSubsystem.scoreAlgae();
        },
        () -> {
          m_algaeIntakeSubsystem.resetIntake();
          m_algaeIntakeSubsystem.setPivotVelocity(0);
        }
      )
    );

    // m_driverController.button(1).onFalse(
    //   new InstantCommand(() -> {
    //     if (m_driverController.button(6).getAsBoolean()) {
    //       Optional<Pose2d> targetPose = Limelight.getScoringPose(
    //         m_swerveDriveSubsystem.getSwerveDrive().getPose(), 
    //         OperatorConstants.kScoreRightOffset,
    //         Meter
    //       );
    //       targetPose.ifPresent((Pose2d pose) -> {
    //         m_swerveDriveSubsystem.setTargetPose(pose);
    //         topSlice.setColor(0, 255, 0);
    //       });
    //     } else if (m_driverController.button(5).getAsBoolean()) {
    //       Optional<Pose2d> targetPose = Limelight.getScoringPose(
    //         m_swerveDriveSubsystem.getSwerveDrive().getPose(), 
    //         OperatorConstants.kScoreLeftOffset,
    //         Meter
    //       );
    //       targetPose.ifPresent((Pose2d pose) -> {
    //         m_swerveDriveSubsystem.setTargetPose(pose);
    //         topSlice.setColor(0, 255, 0);
    //       });
    //     } else {
    //       Optional<Pose2d> targetPose = Limelight.getScoringPose(
    //         m_swerveDriveSubsystem.getSwerveDrive().getPose(), 
    //         OperatorConstants.kScoreCenterOffset,
    //         Meter
    //       );
    //       targetPose.ifPresent((Pose2d pose) -> {
    //         m_swerveDriveSubsystem.setTargetPose(pose);
    //         topSlice.setColor(0, 255, 0);
    //       });
    //     }
    //   }
    // ));

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
            rightSlice.setMeter(1-ElevatorConstants.top/setHeight, null, Color.kOrange);
            rightSlice.setMeter(1-ElevatorConstants.top/setHeight, null, Color.kRed);
          } else {
            leftSlice.setMeter(ElevatorConstants.top, height, Color.kOrange, null);
            leftSlice.setMeter(ElevatorConstants.top, setHeight, Color.kRed, null);
            rightSlice.setMeter(1-ElevatorConstants.top/setHeight, null, Color.kOrange);
            rightSlice.setMeter(1-ElevatorConstants.top/setHeight, null, Color.kRed);
          }
        }
        m_ledSubsystem.display();
      };
    };
    ledIdleCommand.addRequirements(m_ledSubsystem);

    // ConditionalCommand ledDefaultCommand = new ConditionalCommand(
    //   ledIdleCommand, 
    //   ledElevatorHeightCommand, 
    //   () -> m_elevatorSubsystem.getLevelSetpoint().isPresent() ? m_elevatorSubsystem.getLevelSetpoint().get().height()>10 : true
    // );
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
