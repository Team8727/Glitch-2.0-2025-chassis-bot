// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kElevator;
import frc.robot.Constants.kAlgaeRemover.kPivot.RemoverPositions;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgaeCmd extends Command {
  private final AlgaeRemoverPivot m_pivot;
  private final AlgaeRemoverRollers m_rollers;
  private final double m_setPos;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsystem;

  /** Creates a new removeAlgae. */
  public RemoveAlgaeCmd(
      AlgaeRemoverPivot algaeRemoverPivot, AlgaeRemoverRollers algaeRemoverRollers, double level, Elevator elevator, LEDSubsystem ledSubsystem) {
    m_pivot = algaeRemoverPivot;
    m_rollers = algaeRemoverRollers;
    m_setPos = level;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;
    addRequirements(algaeRemoverPivot, algaeRemoverRollers, elevator); // Add the required subsystems here
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_setPos == 2) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.A2); // TODO: set positions
      m_pivot.setRemoverPos(RemoverPositions.Raised.getOutputRotations()); // TODO: set positions
    } else if (m_setPos == 3) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.A3); // TODO: set positions
      m_pivot.setRemoverPos(RemoverPositions.Raised.getOutputRotations()); // TODO: set positions
    }
    m_rollers.spinnnnnnn(); // TODO: set speed

    if (m_setPos == 2){
      m_ledSubsystem.setPatternForDuration(m_ledSubsystem.blue, 2);
    } else if (m_setPos == 3){
      m_ledSubsystem.setPatternForDuration(m_ledSubsystem.ace, 2);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
