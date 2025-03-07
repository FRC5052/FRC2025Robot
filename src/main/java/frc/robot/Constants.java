// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kLEDPort = 1; //TODO change to whatever pwm port the led ends up using
    public static final int kLEDLength = 100;
    public static final Transform2d kScoreOffset = new Transform2d(1,0, new Rotation2d(Math.PI));
  }
  public static class ElevatorConstants {
    // Motor CAN IDs
    public static final int kElevatorMotorID = 13;
    public static final int kFollowerMotorID = 14;
    // PID Constants
    public static final double kP = 0.25;
    public static final double kI = 0;
    public static final double kD = 0;
    // Feedforward Constants
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kMaxVelocity = 1920.0;
    public static final double kMaxAcceleration = 960.0;

    // Zero and max position of motor (rotations)
    public static final double bottom = 0;
    public static final double top = 53.5;

    public static final int kLimitSwitchPort = 0;

    // Gear ratio: ~8.53
    public static final double gearRatio = 8.53;

  }

  //TODO: set these later
  public static class ClawConstants {
    // Motor CAN IDs
    public static final int kPivotMotorID = 16; 
    public static final int kIntakeMotorID = 17;
    public static final int kLimitSwitchPort = 1;
    // PID Constants
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kMaxVelocity = 0;
    public static final double kMaxAcceleration = 0;
    public static final double kIntakeVelocity = 0.2;
    // Claw Positions
    public static final double kIdlePosition = 0;
    public static final double kScorePosition = 0;
  }

  public static class AlgaeIntakeConstants {
    // Motor CAN IDs
    public static final int kPivotMotorID = 18; 
    public static final int kIntakeMotorID = 19;
    public static final int kLimitSwitchPort = 0;
    // PID Constants
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kMaxVelocity = 0;
    public static final double kMaxAcceleration = 0;
    public static final double kIntakeVelocity = 0;
    // Claw Positions
    public static final double kIdlePosition = 0;
    public static final double kScorePosition = 0;
  }
}
