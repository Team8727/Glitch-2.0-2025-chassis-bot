// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class LEDSubsystem extends SubsystemBase { // Fixed class name

  private AddressableLED lightStrip;
  private AddressableLEDBuffer stripBuffer;
  private LEDPattern currentPattern;
  
  // For some reason the Color.k____ values are not working, so I re-defined them here. Red works fine though.
  // I think the blue and green values got flipped somehow.
  private Color m_green = new Color(0, 0, 255);
  private Color m_blue = new Color(0, 255, 0);
  private Color m_purple = new Color(255, 255, 0);
  // Define LED Patterns

  // Blinking red pattern
  public LEDPattern red = LEDPattern.solid(Color.kRed).blink(Second.of(0.5));

  // Rainbow pattern with a scrolling mask
  private LEDPattern rainbowBase = LEDPattern.rainbow(256, 128)
    .scrollAtRelativeSpeed(Percent.per(Second).of(25));
  private LEDPattern rainbowMask = LEDPattern.steps(
      Map.of(
        0.0, Color.kWhite,
        0.25, Color.kBlack,
        0.75, Color.kWhite))
    .scrollAtRelativeSpeed(Percent.per(Second).of(25));
  public LEDPattern rainbow = rainbowBase.reversed().mask(rainbowMask);

  // Blue gradient pattern with a scrolling mask
  public LEDPattern blue = LEDPattern.gradient(
    LEDPattern.GradientType.kContinuous, m_blue, m_green)
    .scrollAtRelativeSpeed(Percent.per(Second).of(25));

  // Green to purple gradient pattern
  public LEDPattern ace = LEDPattern.gradient(
    GradientType.kContinuous, m_green, m_purple)
    .scrollAtRelativeSpeed(Percent.per(Second).of(25));

  public LEDPattern colorCheck = LEDPattern.solid(m_purple);

  // Green pattern that breathes
  public LEDPattern green = LEDPattern.solid(m_green).breathe(Second.of(2));




  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    // LED setup and port configuration
    lightStrip = new AddressableLED(0); // Correct PWM port
    stripBuffer = new AddressableLEDBuffer(108); // Correct LED count

    lightStrip.setLength(stripBuffer.getLength());

    // Set a default pattern (White Solid) to ensure LEDs are not blank initially
    currentPattern = LEDPattern.solid(Color.kBlack);
    currentPattern.applyTo(stripBuffer);
    lightStrip.setData(stripBuffer);
    lightStrip.start();
  }

  public void setPattern(LEDPattern pattern) {
    currentPattern = pattern;
    System.out.println("Pattern set to: " + pattern);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (currentPattern != null) {
      currentPattern.applyTo(stripBuffer);
      lightStrip.setData(stripBuffer);
    }
  }
}
