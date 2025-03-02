// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployCoralCmd extends Command {
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final LEDSubsystem m_ledSubsytem;
  private final ElevatorPosition m_level;

  /** Creates a new coralDeployer. */
  public DeployCoralCmd(Coral coral, ElevatorPosition level, Elevator elevator, LEDSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies
    m_coral = coral;
    m_ledSubsytem = ledSubsystem;
    m_elevator = elevator;
    m_level = level;
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coral.setOuttakeSpeedDuty(.5);
  }

  // Called every time the scheduler runs while the command is scheduled

  boolean sensedCoral = true;

  @Override
  public void execute() {
    m_ledSubsytem.setPatternForDuration(m_ledSubsytem.coralPickup.reversed(), 2);

    if (!m_coral.frontCoralSensor.isPressed() && sensedCoral == true) {
      m_coral.setOutakePos(m_coral.frontMotor.getEncoder().getPosition()+1);
      sensedCoral = false;
    }

    // TODO: this is stupid vvvvvvvvvvvvvvvvvvvvvvv ok maybe not actualy
    if (!m_coral.frontCoralSensor.isPressed() && sensedCoral == false) {
      new Thread(() -> {
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
        m_elevator.setElevatorHeightMotionProfile(m_level);
        sensedCoral = true;
        this.cancel();
        Thread.currentThread().interrupt();
      }).start();
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
