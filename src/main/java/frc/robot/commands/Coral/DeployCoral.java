// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Coral.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployCoral extends Command {
  private final Coral m_coral;
  private double m_elevatorGoalPos;

  /** Creates a new coralDeployer. */
  public DeployCoral(Coral coral, double scoreLevel) {
    // Use addRequirements() here to declare subsystem dependencies
    m_coral = coral;
    m_elevatorGoalPos = scoreLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled
  @Override
  public void execute() {
    if (m_elevatorGoalPos == 0) {
      // TODO: move elevator to position
      m_coral.coralOuttake(0);
    } else if (m_elevatorGoalPos == 1) {
      // TODO: move elevator to position
      m_coral.coralOuttake(0);
    } else if (m_elevatorGoalPos == 2) {
      // TODO: move elevator to position
      m_coral.coralOuttake(0);
    } else if (m_elevatorGoalPos == 3) {
      // TODO: move elevator to position
      m_coral.coralOuttake(0);
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
