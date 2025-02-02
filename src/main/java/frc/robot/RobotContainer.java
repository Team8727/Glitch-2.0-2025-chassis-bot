// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.kVision.kPoses;
import frc.robot.commands.AutoAlignCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.PoseEstimatior;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem;
  private final LEDSubsystem m_ledSubsytem;
  private final CommandXboxController m_driverController;
  private final PoseEstimatior m_PoseEstimatior;
  private final AutoAlignCmd m_autoAlign;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(
      SwerveSubsystem swerveSubsystem,
      LEDSubsystem ledSubsystem,
      CommandXboxController driverController,
      PoseEstimatior poseEstimatior,
      AutoAlignCmd autoAlign) {
    m_SwerveSubsystem = swerveSubsystem;
    m_ledSubsytem = ledSubsystem;
    m_driverController = driverController;
    m_PoseEstimatior = poseEstimatior;
    m_autoAlign = autoAlign;

    // joystickOperated();

    // Configure the trigger bindings
    configureBindings();
  }

  private void joystickOperated() {
    m_SwerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(
        m_SwerveSubsystem,
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> true));
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

    m_driverController.b().onTrue(m_autoAlign.align(kPoses.blueFrontLeft));
      // .until(() -> 
      //     m_PoseEstimatior.get2dPose()
      //       .getTranslation()
      //     .getDistance(
      //       kPoses.blueFrontLeft.getTranslation())
      //     < 0.1 
      //   && 
      //     m_PoseEstimatior.get2dPose()
      //       .getRotation()
      //     .minus(
      //       kPoses.blueFrontLeft.getRotation())
      //         .getDegrees()
      //     < 0.1)
      // .andThen(() -> joystickOperated())
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
