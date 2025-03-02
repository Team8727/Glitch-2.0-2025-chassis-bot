// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Coral.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReindexCoralCmd extends Command {
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsystem;

  /** Creates a new ReindexCoralCmd. */
  public ReindexCoralCmd(Coral coral, Elevator elevator, LEDSubsystem ledSubsystem) {
    m_coral = coral;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;
    addRequirements(coral, elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coral.setIntakeSpeedDuty(.5);
    m_coral.setIntakeSpeedDuty(-.5);
    m_coral.setIntakeSpeedDuty(.5);
    m_coral.setIntakeSpeedDuty(-.5);
    m_coral.setIntakeSpeedDuty(.5);
    m_coral.setIntakeSpeedDuty(-.5);
    m_coral.setIntakeSpeedDuty(0);
    new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsystem);
    this.cancel();
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
