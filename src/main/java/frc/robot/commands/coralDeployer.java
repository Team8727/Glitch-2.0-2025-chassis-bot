// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.kCoralIntake;
import frc.robot.subsystems.coral.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class coralDeployer extends Command {
  private Coral m_coral;
  private double m_elevatorPos;
  private double flywheelSpeed;
  /** Creates a new coralDeployer. */
  public coralDeployer(Coral coral) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coral = coral;
  }

  private Command coralIntake(double speed) {
    return new RunCommand(
      () -> m_coral.setIntakeSpeed(kCoralIntake.kRollers.intakeSpeed))
      .until(() -> m_coral.backCoralSensor.get())
      .andThen(
        new ParallelCommandGroup(
          new RunCommand(() -> m_coral.setIntakeSpeed(flywheelSpeed)),
          new RunCommand(() -> m_coral.setIntakeSpeed(flywheelSpeed)))
        .until(() -> !m_coral.backCoralSensor.get()));
  }

  private Command coralOuttake(double speed) {
    return new RunCommand(() -> m_coral.setOuttakeSpeed(kCoralIntake.kRollers.outtakeSpeed));
  }

  public Command coralDeploy() {
    flywheelSpeed = 0;
    if (m_elevatorPos == 0) {
      flywheelSpeed = 0; // TODO: this should be set later
    } 
    else if (m_elevatorPos == 1 || m_elevatorPos == 2) {
      flywheelSpeed = 0; // TODO: this should be set later
    }
    else if (m_elevatorPos == 3) {
      flywheelSpeed = 0; // TODO: this should be set later
    }

    if (!m_coral.backCoralSensor.get() && !m_coral.frontCoralSensor.get()) {
      return coralIntake(flywheelSpeed);
    } 
    else if (!m_coral.backCoralSensor.get() && m_coral.frontCoralSensor.get()) {
      return coralOuttake(flywheelSpeed);
    } 
    else {
      return new RunCommand(() -> m_coral.stopCoral());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorPos = getElevatorPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
