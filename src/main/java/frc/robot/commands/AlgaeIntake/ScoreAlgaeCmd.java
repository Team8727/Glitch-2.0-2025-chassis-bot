// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntake;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreAlgaeCmd extends Command {
  /** Creates a new ScoreAlgaeProcessorCmd. */
  private final LEDSubsystem m_ledSubsystem;

  AlgaeIntakePivot m_algaeIntakePivot;
  AlgaeIntakeRollers m_algaeIntakeRollers;

  public ScoreAlgaeCmd(
      AlgaeIntakePivot algaeIntakePivot, AlgaeIntakeRollers algaeIntakeRollers, LEDSubsystem ledSubsystem) {
    this.m_algaeIntakePivot = algaeIntakePivot;
    this.m_algaeIntakeRollers = algaeIntakeRollers;
    m_ledSubsystem = ledSubsystem;
    addRequirements(algaeIntakePivot, algaeIntakeRollers); // Add the required subsystems here
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the intake to score position, score the algae by running rollers, and then set the intake to home position.
    m_algaeIntakePivot.setPositionTrapazoidal(kAlgaeIntakePivot.IntakePosition.SCORE);
    m_algaeIntakeRollers.isMoving = true;
    m_algaeIntakeRollers.rollerPID.setReference(-1, ControlType.kDutyCycle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ledSubsystem.setPatternForDuration(m_ledSubsystem.algaePickup.reversed(), 2);

    if (!m_algaeIntakeRollers.getAlgaeCheck()) {
      m_algaeIntakeRollers.rollerPID.setReference(0, ControlType.kDutyCycle);
      m_algaeIntakePivot.setPositionTrapazoidal(kAlgaeIntakePivot.IntakePosition.SCORE);
      m_algaeIntakeRollers.isMoving = false;
      
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Go to home position (in robot) after scoring
    m_algaeIntakePivot.setPositionTrapazoidal(kAlgaeIntakePivot.IntakePosition.HOME);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
