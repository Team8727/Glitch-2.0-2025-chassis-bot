// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.node.DoubleNode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.AlgaeRemover.AlgaeRemoverRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class removeAlgae extends Command {
  private AlgaeRemoverPivot m_pivot;
  private AlgaeRemoverRollers m_rollers;
  private double m_setPos;
  /** Creates a new removeAlgae. */
  public removeAlgae(AlgaeRemoverPivot AlgaeRemoverPivot, AlgaeRemoverRollers AlgaeRemoverRollers, double level) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = AlgaeRemoverPivot;
    m_rollers = AlgaeRemoverRollers;
    m_setPos = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_setPos == 2) {
      m_pivot.setRemoverPos(0);// TODO: set positions
    } else if (m_setPos == 3) {
      m_pivot.setRemoverPos(0);// TODO: set positions
    }
    m_rollers.spinnnnnnn();// TODO: set speed
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
