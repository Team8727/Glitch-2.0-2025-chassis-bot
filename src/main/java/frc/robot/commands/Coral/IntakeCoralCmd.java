// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralCmd extends Command {
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsystem;
  /** Creates a new IntakeCoral. */
  public IntakeCoralCmd(Coral coral, Elevator elevator, LEDSubsystem ledSubsystem) {
    m_coral = coral;
    m_elevator = elevator;
    m_ledSubsystem = ledSubsystem;
    addRequirements(coral, elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.getElevatorSetPosition() != kElevator.ElevatorPosition.L1) {
      System.out.println("go to L1");
      this.cancel();
    }
    m_coral.setIntakeSpeedDuty(.5);
  }

  boolean sensedCoral = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ledSubsystem.setPatternForDuration(m_ledSubsystem.coralPickup, 2);

    if (m_coral.backCoralSensor.isPressed() && sensedCoral == false) {
      m_coral.setIntakeSpeedDuty(.1);
      m_coral.setOuttakeSpeedDuty(.1);
      sensedCoral = true;
    } 

    if (!m_coral.backCoralSensor.isPressed() && sensedCoral == true) {
      m_coral.holdPosition();
      sensedCoral = false;
      this.cancel();
    }

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
