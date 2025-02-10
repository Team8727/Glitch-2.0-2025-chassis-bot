// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Coral.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployCoral extends Command {
  private final Coral m_coral;
  private final Elevator m_elevator;
  private double m_scoreLevel;

  /** Creates a new coralDeployer. */
  public DeployCoral(Coral coral, double scoreLevel, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies
    m_coral = coral;
    m_elevator = elevator;
    m_scoreLevel = scoreLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    if (m_scoreLevel == 1) {
      m_elevator.setElevatorHeight(kElevator.ElevatorPosition.L1);
      m_coral.coralOuttake(kCoral.coraldeploySpeedL1);
    } else if (m_scoreLevel == 2) {
      m_elevator.setElevatorHeight(kElevator.ElevatorPosition.L2);
      m_coral.coralOuttake(kCoral.coraldeploySpeedL2);
    } else if (m_scoreLevel == 3) {
      m_elevator.setElevatorHeight(kElevator.ElevatorPosition.L3);
      m_coral.coralOuttake(kCoral.coraldeploySpeedL3);
    } else if (m_scoreLevel == 4) {
      m_elevator.setElevatorHeight(kElevator.ElevatorPosition.L4);
      m_coral.coralOuttake(kCoral.coraldeploySpeedL4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
