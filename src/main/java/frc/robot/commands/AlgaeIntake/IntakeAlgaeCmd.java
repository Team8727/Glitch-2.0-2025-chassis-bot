// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgaeCmd extends Command {
  /** Creates a new GroundIntakeAlgae. */
  private final AlgaeIntakePivot m_algaeIntakePivot;
  private final AlgaeIntakeRollers m_algaeIntakeRollers;
  private final LEDSubsystem m_ledSubsystem;

  public IntakeAlgaeCmd(
      AlgaeIntakePivot algaeIntakePivot, AlgaeIntakeRollers algaeRemoverPivot, LEDSubsystem ledSubsystem) {
    m_algaeIntakePivot = algaeIntakePivot;
    m_algaeIntakeRollers = algaeRemoverPivot;
    m_ledSubsystem = ledSubsystem;
    addRequirements(algaeIntakePivot, algaeRemoverPivot); // Add the required subsystems here
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("moving");
    m_algaeIntakeRollers.isMoving = true;
    m_algaeIntakePivot.setPositionTrapazoidal(kAlgaeIntakePivot.IntakePosition.DOWN);
    m_algaeIntakeRollers.setRollerSpeedDuty(.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ledSubsystem.setPatternForDuration(m_ledSubsystem.algaePickup, 2);

    if (m_algaeIntakeRollers.getAlgaeCheck()) {
      m_algaeIntakeRollers.isMoving = false;
      m_algaeIntakePivot.setPositionTrapazoidal(kAlgaeIntakePivot.IntakePosition.HOME);
      
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Set the intake rollers to idle pull in voltage
    // Go back to home position
    // m_algaeIntakePivot.setIntakePivotPosition(kAlgaeIntakePivot.IntakePosition.HOME);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Finish when algae is detected
    return false;
  }
}
