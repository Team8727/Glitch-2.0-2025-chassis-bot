// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundIntakeAlgaeCmd extends Command {
  /** Creates a new GroundIntakeAlgae. */
  AlgaeIntakePivot m_algaeIntakePivot;

  AlgaeIntakeRollers m_algaeIntakeRollers;

  public GroundIntakeAlgaeCmd(
      AlgaeIntakePivot algaeIntakePivot, AlgaeIntakeRollers algaeRemoverPivot) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_algaeIntakePivot = algaeIntakePivot;
    this.m_algaeIntakeRollers = algaeRemoverPivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Set the intake pivot to the ground position and
    Commands.sequence(
        m_algaeIntakePivot.setIntakePivotPosition(kAlgaeIntakePivot.intakePivotDownPosition),
        m_algaeIntakeRollers.intake(),
        m_algaeIntakePivot.setIntakePivotPosition(kAlgaeIntakePivot.intakePivotScorePosition));

    // Set the intake rollers to idle pull in voltage
    m_algaeIntakeRollers.setRollerSpeed(kAlgaeIntakePivot.idleAlgaeIntakeVoltage);
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
