// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Coral.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RejectCoralCmd extends Command {
  private final Coral m_coral;

  /** Creates a new ReindexCoralCmd. */
  public RejectCoralCmd(Coral coral) {
    m_coral = coral;
    addRequirements(coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new Thread(() -> {
      m_coral.setIntakeSpeedDuty(-.2);
      try {
        Thread.sleep(300);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        e.printStackTrace();
      }
      m_coral.setIntakeSpeedDuty(0);
      this.cancel();
      Thread.currentThread().interrupt();
    }).start();

    // m_coral.setIntakeSpeedDuty(-.1);
    // CustomCommands.waitCommand(.1, () -> m_coral.setIntakeSpeedDuty(.1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
