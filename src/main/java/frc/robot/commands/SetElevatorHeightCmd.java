// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.commands.Coral.IntakeCoralCmd;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorHeightCmd extends Command {
  private final Elevator m_elevator;
  private ElevatorPosition m_scoreLevel;
  private final LEDSubsystem m_ledSubsystem;
  private final Coral m_coral;


  /** Creates a new SetEvevatorHeightCmd. */
  public SetElevatorHeightCmd(ElevatorPosition scoreLevel, Elevator elevator, Coral coral, LEDSubsystem ledSubsystem) {

    m_scoreLevel = scoreLevel;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;
    m_coral = coral;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, coral, ledSubsystem);

    this.beforeStarting(() -> {
      m_coral.elevatorUp = true;
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsystem);
    }, m_coral);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_coral.setIntakeSpeedDuty(.5);
      this.cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_coral.getFrontCoralSensor()) {
      m_coral.setIntakeSpeedDuty(0);
      m_coral.holdPosition();
      m_elevator.setElevatorHeightMotionProfile(m_scoreLevel);
    }
    m_ledSubsystem.setPatternForDuration(m_ledSubsystem.coralPickup.reversed(), 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_scoreLevel.getOutputRotations() - m_elevator.getElevatorHeight()) < 0.1;
  }
}
