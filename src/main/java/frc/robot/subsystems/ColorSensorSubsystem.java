// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensorSubsystem extends SubsystemBase {
  private final I2C.Port i2cPort;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher;
  /** Creates a new ExampleSubsystem. */
  public ColorSensorSubsystem() {
    i2cPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes20bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain18x);
    m_colorMatcher = new ColorMatch();
    m_colorMatcher.setConfidenceThreshold(0.75);
    m_colorMatcher.addColorMatch(Color.kBlack);
    m_colorMatcher.addColorMatch(Color.kOrange);
    // m_colorMatcher.addColorMatch(Color.kRed);
    // m_colorMatcher.addColorMatch(Color.kBlue);
    // m_colorMatcher.addColorMatch(Color.kYellow);
    // m_colorMatcher.addColorMatch(Color.kGreen); 
     
  }

  public RawColor getRawColor() {
    return m_colorSensor.getRawColor();
  }

  public Color getColor() {
    return m_colorSensor.getColor();
  }

  public Color getMatchedColor() {
    ColorMatchResult match = m_colorMatcher.matchClosestColor(getColor());
    System.out.println(getColor());
    System.out.println(match.color);
    return match.color;
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
