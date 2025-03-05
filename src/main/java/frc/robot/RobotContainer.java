// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controller.Controller;
import frc.robot.controller.DefaultTeleopControllerBindings;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.Coral;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem;
  private final AlgaeIntakePivot m_AlgaeIntakePivot;
  private final AlgaeIntakeRollers m_AlgaeIntakeRollers;
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot;
  private final AlgaeRemoverRollers m_AlgaeRemoverRollers;
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsytem;
  private final Autos m_Autos;
  private boolean m_elevatorSpeedControl;
  private final Controller m_controller = new Controller();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(
      SwerveSubsystem swerveSubsystem,
      AlgaeIntakePivot AlgaeIntakePivot,
      AlgaeIntakeRollers AlgaeIntakeRollers,
      AlgaeRemoverPivot AlgaeRemoverPivot,
      AlgaeRemoverRollers AlgaeRemoverRollers,
      Coral coral,
      Elevator elevator,
      LEDSubsystem ledSubsystem,
      Autos autos,
      boolean elevatorSpeedControl
      ) {
    m_SwerveSubsystem = swerveSubsystem;
    m_AlgaeIntakePivot = AlgaeIntakePivot;
    m_AlgaeIntakeRollers = AlgaeIntakeRollers;
    m_AlgaeRemoverPivot = AlgaeRemoverPivot;
    m_AlgaeRemoverRollers = AlgaeRemoverRollers;
    m_coral = coral;
    m_elevator = elevator;
    m_ledSubsytem = ledSubsystem;
    m_Autos = autos;
    m_elevatorSpeedControl = elevatorSpeedControl;
  }

  public void teleopInit() {
    m_controller.applyBindings(new DefaultTeleopControllerBindings(
      m_SwerveSubsystem,
      m_AlgaeIntakePivot,
      m_AlgaeIntakeRollers,
      m_coral,
      m_elevator,
      m_ledSubsytem,
      m_elevatorSpeedControl
    ));
  }

  public void autonomousInit() {
    m_controller.clearBindings();
  }
}
