// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kAlgaeRemover.kPivot.RemoverPositions;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgaeCmd extends Command {
  private final AlgaeRemoverPivot m_pivot;
  private final AlgaeRemoverRollers m_rollers;
  private final ElevatorPosition m_setPos;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsystem;

  /** Creates a new removeAlgae. */
  public RemoveAlgaeCmd(
      AlgaeRemoverPivot algaeRemoverPivot, AlgaeRemoverRollers algaeRemoverRollers, ElevatorPosition setPos, Elevator elevator, LEDSubsystem ledSubsystem) {
    m_pivot = algaeRemoverPivot;
    m_rollers = algaeRemoverRollers;
    m_setPos = setPos;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;
    addRequirements(algaeRemoverPivot, algaeRemoverRollers, elevator); // Add the required subsystems here
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.getElevatorSetPosition() == ElevatorPosition.A2 || m_elevator.getElevatorSetPosition() == ElevatorPosition.A3) {
      System.out.println("test");
      m_pivot.setPositionTrapazoidal(RemoverPositions.Raised); // TODO: set positions
      m_rollers.spinnnnnnn(); // TODO: set speed
      new Thread(() -> {
          try {
            Thread.sleep(500);
            m_pivot.setPositionTrapazoidal(RemoverPositions.Stowed);
            m_rollers.setRemoverRollerSpeed(0);
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            e.printStackTrace();
          }
          this.cancel();
          Thread.currentThread().interrupt();    
        }).start();
      } else {
      m_elevator.setElevatorHeightMotionProfile(m_setPos);
      this.cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ledSubsystem.setPatternForDuration(m_ledSubsystem.blue, 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
