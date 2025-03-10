// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Robot.isRedAlliance;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator.Elevator;

public class LEDSubsystem extends SubsystemBase { // Fixed class name

  private AddressableLED lightStrip;
  private AddressableLEDBuffer stripBuffer;
  private AddressableLEDBufferView leftSide;
  private AddressableLEDBufferView rightSide;
  private LEDPattern currentPattern;

  private Elevator m_elevator;

  // For some reason the Color.k____ values are not working, so I re-defined them here. Red works fine though.
  // I think the blue and green values got flipped somehow.
  private Color m_green = new Color(0, 0, 255);
  private Color m_blue = new Color(0, 255, 0);
  private Color m_purple = new Color(255, 255, 0);
  private Color m_orange = new Color(255, 0, 165);
  private Color m_yellow = new Color(255, 0, 255);
  private Color m_pink = new Color(255, 203, 192);
  // Define LED Patterns

  // Blinking red pattern
  public LEDPattern red = LEDPattern.solid(Color.kRed)
  .blink(Second.of(0.5));

  // Rainbow pattern with a scrolling mask
  public LEDPattern rainbow = LEDPattern.rainbow(
    256, 
    256)
    .scrollAtRelativeSpeed(
      Percent.per(Second).of(15))
      .reversed()
      .mask(
        LEDPattern.steps(
          Map.of(
              0.0, Color.kWhite,
              0.25, Color.kBlack,
              0.75, Color.kWhite))
      .scrollAtRelativeSpeed(
        Percent.per(Second).of(20)));

  // Blue gradient pattern with a scrolling mask
  public LEDPattern blue =
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, m_blue, m_green)
          .scrollAtRelativeSpeed(
            Percent.per(Second).of(15));

  // Green to purple gradient pattern
  public LEDPattern ace =
      LEDPattern.gradient(GradientType.kContinuous, m_green, m_purple)
          .scrollAtRelativeSpeed(
            Percent.per(Second).of(15));

  public LEDPattern green = LEDPattern.solid(m_green);
  // Elevator progress bar pattern
  public LEDPattern elevatorProgressBase = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    m_green, 
    m_yellow, 
    m_orange, 
    Color.kRed);
  public LEDPattern elevatorProgressMap = LEDPattern.progressMaskLayer(
    () -> m_elevator.getElevatorHeight() / kElevator.ElevatorPosition.L4.getOutputRotations());
  public LEDPattern elevatorProgress = elevatorProgressBase.mask(elevatorProgressMap);
  // Coral pickup pattern
  public LEDPattern coralPickup = LEDPattern.gradient(
    GradientType.kDiscontinuous, 
    m_green, 
    m_pink, 
    m_yellow, 
    Color.kRed)
      .blink(Second.of(0.5));

  // Algae pickup pattern
  public LEDPattern algaePickup = LEDPattern.gradient(
    GradientType.kDiscontinuous,
     m_green,
     m_purple,
     m_orange,
     Color.kRed)
      .blink(Second.of(0.5));
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(Elevator elevator) {
    // LED setup and port configuration
    lightStrip = new AddressableLED(5); // Correct PWM port
    stripBuffer = new AddressableLEDBuffer(114); // Correct LED count
    leftSide = new AddressableLEDBufferView(stripBuffer, 0, 57);
    rightSide = new AddressableLEDBufferView(stripBuffer, 57, 113).reversed();

    lightStrip.setLength(stripBuffer.getLength());

    // // Set a default pattern (White Solid) to ensure LEDs are not blank initially
    currentPattern = LEDPattern.solid(m_green);
    lightStrip.setData(stripBuffer);
    lightStrip.start();

    m_elevator = elevator;
  }

  public void setPattern(LEDPattern pattern) {
    currentPattern = pattern;
    //System.out.println("Pattern set to: " + pattern);
  }

  /**
   * Sets a pattern for a duration in seconds
   * @param pattern the LEDPattern to apply
   * @param seconds the duration in seconds to set the pattern to before turning the strip off
   */
  public void setPatternForDuration(LEDPattern pattern, double seconds) {
    //Command Composition for duration pattern
    new RunCommand(
      () -> currentPattern = pattern)
    .withTimeout(seconds);

    //Notify of duration pattern
    //System.out.println("Pattern was set to: " + pattern + " for " + seconds + " seconds");
  }

  public void turnLEDsOff() {
    currentPattern = LEDPattern.solid(Color.kBlack);
    //System.out.println("Pattern set to: " + LEDPattern.solid(Color.kBlack));
  }

  public void combinePatterns(LEDPattern leftPattern, LEDPattern rightPattern) {
    leftPattern.applyTo(leftSide);
    rightPattern.applyTo(rightSide);
    //System.out.println("Pattern set to: " + pattern1.combine(pattern2));
  }

  public void combinePatternsForDuration(LEDPattern leftPattern, LEDPattern rightPattern, double seconds) {
    //Command Composition for duration pattern
    new RunCommand(
      () -> {
        leftPattern.applyTo(leftSide);
        rightPattern.applyTo(rightSide);
      })
    .withTimeout(seconds);

    //Notify of duration pattern
    //System.out.println("Pattern was set to: " + pattern1.combine(pattern2) + " for " + seconds + " seconds");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (currentPattern != null) {
      currentPattern.atBrightness(Percent.of(40)).applyTo(leftSide);
      currentPattern.atBrightness(Percent.of(40)).applyTo(rightSide);
      lightStrip.setData(stripBuffer);
    }
  }
}
