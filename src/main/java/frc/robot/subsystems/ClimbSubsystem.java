package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    // private SparkMax climbMotor;

    // private ProfiledPIDController pid = new ProfiledPIDController(
    //     0.3, 
    //     0.0, 
    //     0.0, 
    //     new Constraints(
    //         256.0, 
    //         512.0
    //     )
    // );


    public ClimbSubsystem() {
        // super();
        // climbMotor = new SparkMax(15, MotorType.kBrushless);
        // this.setPositionSetpoint(ClimbPosition.Idle);
    }

    // private double getActualPosition() {
    //     return this.climbMotor.getEncoder().getPosition();
    // }

    // public void setPositionSetpoint(ClimbPosition position) {
    //     this.pid.setGoal(position.position);
    // }

    // @Override
    // public void periodic() {
    //     // System.out.println(this.climbMotor.getEncoder().getPosition());
    //     this.climbMotor.set(this.pid.calculate(this.getActualPosition()));
    // }

    // public enum ClimbPosition {
    //     Idle(0.0),
    //     Score(128.0);

    //     private final double position;

    //     private ClimbPosition(double position) {
    //         this.position = position;
    //     }

    //     public ClimbPosition next() {
    //         switch (this) {
    //             case Idle:
    //                 return Score;
    //             case Score:
    //                 return Idle;
    //             default:
    //                 return null;
    //         }
    //     }

    //     public ClimbPosition prev() {

    //         switch (this) {
    //             case Idle:
    //                 return Score;
    //             case Score:
    //                 return Idle;
    //             default:
    //                 return null;

    //         }
    //     }

    //     // public void setPositionSetpoint(ClimbPosition position) {
    //     //     this.position = position;
    //     //     pid.setGoal(position.position());
    
    //     // }
    // }
}