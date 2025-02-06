// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakeRollers;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgaeCmd extends Command {
  /** Creates a new GroundIntakeAlgae. */
  private final AlgaeIntakePivot m_algaeIntakePivot;

  private final AlgaeIntakeRollers m_algaeIntakeRollers;

  public IntakeAlgaeCmd(
      AlgaeIntakePivot algaeIntakePivot, AlgaeIntakeRollers algaeRemoverPivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_algaeIntakePivot = algaeIntakePivot;
    m_algaeIntakeRollers = algaeRemoverPivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public Command intake() {
    return new RunCommand(() -> m_algaeIntakeRollers.setRollerSpeed(kAlgaeIntakeRollers.intakeSpeed))
        .until(() -> m_algaeIntakeRollers.getAlgaeCheck())
        .andThen(
            new RunCommand(() -> m_algaeIntakeRollers.setRollerSpeed(kAlgaeIntakeRollers.intakeSpeed))
                .withTimeout(0.5)) // TODO: this additional time may have to be modified or removed
        .finallyDo(() -> m_algaeIntakeRollers.setRollerSpeed(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Set the intake pivot to the ground position and
    m_algaeIntakePivot.setIntakePivotPosition(kAlgaeIntakePivot.intakePivotDownPosition);
    intake();
    m_algaeIntakePivot.setIntakePivotPosition(kAlgaeIntakePivot.intakePivotScorePosition);
    // Set the intake rollers to idle pull in voltage
    m_algaeIntakeRollers.stopRollers();
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
