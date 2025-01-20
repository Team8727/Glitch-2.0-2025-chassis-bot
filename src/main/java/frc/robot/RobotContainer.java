// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.PoseEstimatior;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.PoseEstimatior;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final LEDSubsytem m_LedSubsytem = new LEDSubsytem();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final PoseEstimatior m_PoseEstimatior = new PoseEstimatior(m_SwerveSubsystem);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_SwerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        m_SwerveSubsystem,
          () -> -m_driverController.getLeftY(),
          () -> -m_driverController.getLeftX(),
          () -> m_driverController.getRightX(),
          () -> true));
      // Configure the trigger bindings
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
    // zero heading
    m_driverController.a().onTrue(new InstantCommand(m_SwerveSubsystem::zeroHeading));
    // x configuration
    m_driverController.x().toggleOnTrue(m_SwerveSubsystem.XPosition());
    // This sets all of the LEDs to the color green. Placeholder command.
    // m_driverController.y().toggleOnTrue(m_LedSubsytem.runPattern(LEDPattern.solid(Color.kGreen)).withName("Full Green"));

    // These triggers all reference commands already explained in LEDSubsystem. The names are just for fun.
    m_driverController.y().toggleOnTrue(m_LedSubsytem.redLight().withName("Red Light"));
    m_driverController.b().toggleOnTrue(m_LedSubsytem.scrollingGradient().withName("The Blues"));
    m_driverController.leftBumper().toggleOnTrue(m_LedSubsytem.maskedRainbow().withName("Running Rainbow"));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
