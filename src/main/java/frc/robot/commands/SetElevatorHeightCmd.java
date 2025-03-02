// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.Constants.kElevator;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorHeightCmd extends Command {
  private final Elevator m_elevator;
  private double m_scoreLevel;
  private final LEDSubsystem m_ledSubsytem;


  /** Creates a new SetEvevatorHeightCmd. */
  public SetElevatorHeightCmd(double scoreLevel, Elevator elevator, LEDSubsystem ledSubsystem) {
    m_scoreLevel = scoreLevel;
    m_elevator = elevator;
    m_ledSubsytem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_scoreLevel == 1) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L1);
    } else if (m_scoreLevel == 2) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L2);
    } else if (m_scoreLevel == 3) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L3);
    } else if (m_scoreLevel == 4) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L4);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ledSubsytem.setPatternForDuration(m_ledSubsytem.coralPickup.reversed(), 2);
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
