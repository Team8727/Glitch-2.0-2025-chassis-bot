// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgaeCmd extends Command {
  /** Creates a new GroundIntakeAlgae. */
  private final AlgaeIntakePivot m_algaeIntakePivot;

  private final AlgaeIntakeRollers m_algaeIntakeRollers;

  public IntakeAlgaeCmd(
      AlgaeIntakePivot algaeIntakePivot, AlgaeIntakeRollers algaeRemoverPivot) {
    m_algaeIntakePivot = algaeIntakePivot;
    m_algaeIntakeRollers = algaeRemoverPivot;
    addRequirements(algaeIntakePivot, algaeRemoverPivot); // Add the required subsystems here
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Set the intake pivot to the ground position and
    m_algaeIntakePivot.setIntakePivotPosition(kAlgaeIntakePivot.IntakePosition.DOWN);
    m_algaeIntakeRollers.intake();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Set the intake rollers to idle pull in voltage
    m_algaeIntakeRollers.setRollerSpeed(kAlgaeIntakePivot.idleAlgaeIntakeVoltage);

    // Go back to home position
    m_algaeIntakePivot.setIntakePivotPosition(kAlgaeIntakePivot.IntakePosition.HOME);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Finish when algae is detected
    return m_algaeIntakeRollers.intake().isFinished();
  }
}
