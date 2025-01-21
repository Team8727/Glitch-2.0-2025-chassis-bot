// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.nio.Buffer;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDSubsytem extends SubsystemBase {

  // IMPORTANT: Turns out we can't use commands normally with LEDs and I don't know why. It will not display any errors but it also won't work.

  AddressableLED lightStrip;
  AddressableLEDBuffer stripBuffer;

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  /** Creates a new LEDSubsytem. */
  public LEDSubsytem() {
        // Look for here for further info on LED stuff: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#led-patterns
    // The density of the LED strip we have on the robot is 60 per meter

        // Here we tell the code what PWM port the LED strip is connected to on the roboRio, so we can actually tell the LEDs what to do.
        lightStrip = new AddressableLED(0); // The 2024 robot's port is 0
        
        // I think the buffer is just so we aren't directly setting the length of the LED, because apparently that is "expensive".
        stripBuffer = new AddressableLEDBuffer(4); // The 2024 robot had 46 LEDs.
        
        // Here we set the length of the strip, get the data from the buffer, and .start() writes the output continously.
        // I honestly don't understand a lot of this so check the docs to be sure.
        lightStrip.setLength(stripBuffer.getLength());

        // LEDPattern red = LEDPattern.solid(Color.kRed);

        LEDPattern rainbowBase = LEDPattern.rainbow(256, 128)
          .mask(LEDPattern.steps(
            Map.of(0, Color.kWhite, 0.5, Color.kBlack)));
        LEDPattern rainbow = rainbowBase.scrollAtRelativeSpeed(Percent.per(Second).of(0.25));

        LEDPattern blueBase = LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kPurple);
        LEDPattern blue = blueBase.scrollAtRelativeSpeed(Percent.per(Second).of(0.25));

        LEDPattern greenBase = LEDPattern.solid(Color.kGreen);
        LEDPattern green = greenBase.breathe(Second.of(2));

        setDefaultCommand(new InstantCommand(() -> LEDPattern.solid(Color.kWhite).applyTo(stripBuffer)));
        // // Write the data to the LED strip
        lightStrip.setData(stripBuffer);
        lightStrip.start();

        m_driverController.y().onTrue(new InstantCommand(() -> rainbow.applyTo(stripBuffer)));
        m_driverController.b().onTrue(new InstantCommand(() -> blue.applyTo(stripBuffer)));
        m_driverController.leftBumper().onTrue(new InstantCommand(() -> LEDPattern.solid(Color.kRed).applyTo(stripBuffer)));
        m_driverController.rightBumper().onTrue(new InstantCommand(() -> green.applyTo(stripBuffer)));
      }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // runPattern(LEDPattern.solid(Color.kWhite));
    lightStrip.setData(stripBuffer);
  }
  // Everything below this line was experimental and did not work. I am keeping it here for reference.
  // ----------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------------------------------------------------------------------------
  // This command significantly lightens my workload for all of the other commands
  // so I don't have to keep applying the data to the buffer manually.
  // public Command runPattern(LEDPattern pattern) {
  //   return run(
  //     () -> pattern
  //     .applyTo(stripBuffer))
  //     .andThen(() -> lightStrip.setData(stripBuffer));
  // }

  // This command is incredibly straightforward. It just sets all the lights to red.
  // public Command redLight() {
  //   LEDPattern red = LEDPattern.solid(Color.kRed);
  //   return runPattern(red);
  // }
  
  // This command calls the gradient method, which can be either continous or discontinous.
  // Continuous is good if you have multiple strips or if you are scrolling, like we are here.
  // After which, we call the .scrollAtRelativeSpeed method, which we do because it will scroll
  // at the exact same speed regardless of the size of the LED strip, which means that while it
  // is less intuitive than the .scrollAtAbsoluteSpeed method, it is more reliable for different situations.
  // public Command scrollingGradient() {
  //   return runPattern(
  //     LEDPattern.gradient(
  //       LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kPurple)
  //       .scrollAtRelativeSpeed(Percent.per(Second).of(0.25)));
  // }

  // Rainbow command! The saturation is the intensity of the colors and the value is how bright they are.
  // 256 is the maximum for both values.
  // The mask is the tricky part. We make a map, which identifies sections of the LED strip as certain colors.
  // The number values indicate where each section of the mask starts, with the "white" section starting at the
  // beginning and the "black" section starting one-third of the way through.
  // What the mask does is it looks at whatever the color of the LED is that is beneath it and it checks it against
  // the mask's color. If it has a similar color value, then it stays. If it has a different one, then it is replaced
  // by the mask. Basically what the code here is doing is showing a small sliver of a rainbow that scrolls through the
  // strip, getting replaced by darkness along the way.
  // public Command maskedRainbow() {
  //   return runPattern(
  //     LEDPattern.rainbow(256, 128)
  //     .mask(LEDPattern.steps(
  //       Map.of(0, Color.kWhite, 0.3, Color.kBlack))
  //     .scrollAtRelativeSpeed(
  //       Percent.per(Second).of(0.25))));
  //}
}
