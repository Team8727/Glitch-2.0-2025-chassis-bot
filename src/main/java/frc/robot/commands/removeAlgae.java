// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class removeAlgae extends Command {
  private final AlgaeRemoverPivot m_pivot;
  private final AlgaeRemoverRollers m_rollers;
  private final double m_setPos;
  private final Elevator m_elevator;

  /** Creates a new removeAlgae. */
  public removeAlgae(
      AlgaeRemoverPivot AlgaeRemoverPivot, AlgaeRemoverRollers AlgaeRemoverRollers, double level, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = AlgaeRemoverPivot;
    m_rollers = AlgaeRemoverRollers;
    m_setPos = level;
    m_elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_setPos == 2) {
      m_elevator.setElevatorHeight(kElevator.ElevatorPosition.A2); // TODO: set positions
      m_pivot.setRemoverPos(0); // TODO: set positions
    } else if (m_setPos == 3) {
      m_elevator.setElevatorHeight(kElevator.ElevatorPosition.A3); // TODO: set positions
      m_pivot.setRemoverPos(0); // TODO: set positions
    }
    m_rollers.spinnnnnnn(); // TODO: set speed
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
