// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.ReplanningConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveDrive.HeadingControlMode;
import frc.robot.swerve.SwerveMotor.SparkMaxSwerveMotor;
import frc.robot.swerve.SwerveEncoder.CANCoderSwerveEncoder;
import frc.robot.swerve.SwerveIMU.NavXSwerveIMU;

public class SwerveDriveSubsystem extends SubsystemBase {
  private SwerveDrive swerveDrive;

  private boolean seeingAprilTag = false;
  private boolean teleopIsFieldCentric = true;
  private MedianFilter poseResetXFilter = new MedianFilter(10);
  private MedianFilter poseResetYFilter = new MedianFilter(10);
  /** Creates a new ExampleSubsystem. */
  public SwerveDriveSubsystem() {

    var currentAlliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : DriverStation.Alliance.Blue;

    SwerveModule.Builder module_cfg = SwerveModule.builder()
      .withDriveGearRatio(6.75)
      .withPivotGearRatio(12.8)
      .withWheelDiameter(0.1016, Meters);

    SparkMaxSwerveMotor.Builder motor_cfg = new SparkMaxSwerveMotor.Builder()
      .withIdleMode(IdleMode.kBrake) 
      .withMotorType(MotorType.kBrushless)
      .withCurrentLimit(40, Amps);

    CANCoderSwerveEncoder.Builder encoder_cfg = new CANCoderSwerveEncoder.Builder();

    this.swerveDrive = SwerveDrive.builder()
      .withIMU(new NavXSwerveIMU.Builder().withPort())
      .withModules(new SwerveModule.Builder[] {
        module_cfg.clone() // Front Right
          .withOffset(new Translation2d(10, -10), Inches)
          .withDriveMotor(motor_cfg.clone().withID(10))
          .withPivotMotor(motor_cfg.clone().withID(11))
          .withAbsoluteEncoder(encoder_cfg.clone().withID(12).withOffset(-57.57+180, Degrees)),
        module_cfg.clone() // Front Left
          .withOffset(new Translation2d(10, 10), Inches)
          .withDriveMotor(motor_cfg.clone().withID(7))
          .withPivotMotor(motor_cfg.clone().withID(8))
          .withAbsoluteEncoder(encoder_cfg.clone().withID(9).withOffset(35.24-180, Degrees)),
        module_cfg.clone() // Back Right
          .withOffset(new Translation2d(-10, -10), Inches)
          .withDriveMotor(motor_cfg.clone().withID(4))
          .withPivotMotor(motor_cfg.clone().withID(5))
          .withAbsoluteEncoder(encoder_cfg.clone().withID(6).withOffset(-124.36+180, Degrees)),
        module_cfg.clone() // Back Left
          .withOffset(new Translation2d(-10, 10), Inches)
          .withDriveMotor(motor_cfg.clone().withID(1))
          .withPivotMotor(motor_cfg.clone().withID(2))
          .withAbsoluteEncoder(encoder_cfg.clone().withID(3).withOffset(27.7-180, Degrees))
      })
      .withHeadingPID(new PIDConstants(4.0, 0.0, 0.2))
      .withModuleDrivePID(new PIDConstants(0.5, 0.0, 0.0))
      .withModulePivotPID(new PIDConstants(1.0, 0.0, 0.0))
      .withInitialPose(currentAlliance.equals(DriverStation.Alliance.Blue) ? 
        new Pose2d(0, 0, Rotation2d.k180deg) : 
        new Pose2d(0, 0, Rotation2d.kZero)
      )
      .build();

    this.setFullSpeed(true);
    double maxDistance = 0.0;
    for (int i = 0; i < this.swerveDrive.getNumSwerveModules(); i++) {
      SwerveModule module = this.swerveDrive.getSwerveModule(i);
      // module.getEncoder().setOffset(MathUtil.inputModulus(this.swerveDrive.getSwerveModule(i).getEncoder().getOffset(Degrees) - 90.0, -180, 180), Degrees);
      // module.getPivotMotor().setCurrentLimit(20, Amps);
      double distance = module.getModuleOffset().getNorm();
      if (maxDistance < distance) maxDistance = distance;
    }

    this.swerveDrive.setPowerDistribution(new PowerDistribution(20, ModuleType.kRev));
    try {
      AutoBuilder.configure(
        this.swerveDrive::getPose, 
        this.swerveDrive::setPose, 
        this.swerveDrive::getActualSpeeds,
        (ChassisSpeeds speeds) -> {
          // double xOutput = -speeds.vxMetersPerSecond/this.swerveDrive.getMaxDriveSpeed(MetersPerSecond);
          // double yOutput = -speeds.vyMetersPerSecond/this.swerveDrive.getMaxDriveSpeed(MetersPerSecond);
          // double hOutput = -speeds.omegaRadiansPerSecond/this.swerveDrive.getMaxTurnSpeed(RadiansPerSecond);

          // System.out.printf("driving %f, %f, %f\n", xOutput, yOutput, hOutput);
          this.swerveDrive.drive(
            new ChassisSpeeds(
              -speeds.vxMetersPerSecond,
              -speeds.vyMetersPerSecond,
              -speeds.omegaRadiansPerSecond
            ),
            false
          );
        }, 
        new PPHolonomicDriveController(
          new PIDConstants(0.5, 0.0, 0.0),
          new PIDConstants(
            swerveDrive.getHeadingController().getP(), 
            swerveDrive.getHeadingController().getI(),
            swerveDrive.getHeadingController().getD()
          )
        ), 
        RobotConfig.fromGUISettings(), 
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, 
        this
      );
    } catch (IOException | ParseException e) {
      // TODO Review whether this is best practice exception handling for this scenario
      System.out.println("IO or Parse exception");
      e.printStackTrace();
    }
  }

  public void resetHeading() {
    this.swerveDrive.zeroHeading();
    // this.swerveDrive.overridePosition(new Pose2d());
  }

  public SwerveDrive getSwerveDrive() {
    return this.swerveDrive;
  }

  public void setFullSpeed(boolean fullSpeed) {
    if (fullSpeed) {
      this.swerveDrive.setMaxDriveSpeed(4.0, MetersPerSecond); // Full drive speed
      this.swerveDrive.setMaxTurnSpeed(0.75, RotationsPerSecond); // Full turn speed
      this.swerveDrive.setMaxDriveAccel(8.0, MetersPerSecondPerSecond);
      this.swerveDrive.setMaxTurnAccel(4.0, RotationsPerSecond.per(Second));
    } else {
      this.swerveDrive.setMaxDriveSpeed(4.0/4, MetersPerSecond); // Non-full drive speed
      this.swerveDrive.setMaxTurnSpeed(0.75/2, RotationsPerSecond); // Non-full turn speed
      this.swerveDrive.setMaxDriveAccel(8.0, MetersPerSecondPerSecond);
      this.swerveDrive.setMaxTurnAccel(4.0, RotationsPerSecond.per(Second));
    }
  }

  public void setTeleopIsFieldCentric(boolean fieldCentric) {
    this.teleopIsFieldCentric = fieldCentric;
  }

  public boolean teleopIsFieldCentric() {
    return this.teleopIsFieldCentric;
  }

  public Command createPathfindingCommand(Pose2d pose) {
    // this.swerveDrive.overrideTargetHeading(-this.swerveDrive.getPoseAngle(Radians), Radians);
    Command command = AutoBuilder.pathfindToPose(
      pose, 
      new PathConstraints(
        this.swerveDrive.getMaxDriveSpeed(MetersPerSecond) / 2.0, 
        this.swerveDrive.getMaxDriveAccel(MetersPerSecondPerSecond) / 8.0, 
        this.swerveDrive.getMaxTurnSpeed(RadiansPerSecond), 
        this.swerveDrive.getMaxTurnAccel(RadiansPerSecond.per(Second))
      )
    );

    command.addRequirements(this);

    return command;
  }

  public Command createFineAdjustCommand(double x, double y, double h, HeadingControlMode mode) {
    Command command = new RunCommand(() -> {
      this.getSwerveDrive().drive(
          x, 
          y, 
          h, 
          mode, 
          false
        );
    }, this);
    return command;
  }

  @Override
  public void periodic() {
    // System.out.println(this.swerveDrive.getTargetHeading());
    {
      Limelight.setRobotYaw(this.swerveDrive.getPoseAngle(Radians), this.swerveDrive.getActualSpeeds().omegaRadiansPerSecond, Radians, RadiansPerSecond);
      var aprilTagPose = Limelight.getFieldCentricRobotPose(Meters, false);
      if (aprilTagPose.isPresent() && this.seeingAprilTag) {
        Rotation2d rotation = new Rotation2d(this.swerveDrive.getActualHeading(Radians));
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          rotation = rotation.plus(Rotation2d.k180deg);
        }
        this.swerveDrive.setPose(new Pose2d(this.poseResetXFilter.calculate(aprilTagPose.get().getX()), this.poseResetYFilter.calculate(aprilTagPose.get().getY()), rotation));
      } else {
        this.poseResetXFilter.calculate(this.swerveDrive.getPoseX(Meters));
        this.poseResetYFilter.calculate(this.swerveDrive.getPoseY(Meters));
      }
      this.seeingAprilTag = aprilTagPose.isPresent();
    }
    this.swerveDrive.update();
    
  }
}
