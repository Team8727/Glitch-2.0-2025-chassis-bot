// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PoseEstimatior;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChoreoPathCmd extends Command {
  SwerveSubsystem m_SwerveSubsystem;
  PoseEstimatior m_PoseEstimatior;
  AutoFactory m_AutoFactory;

  /** Creates a new choreoPath. */
  public ChoreoPathCmd(SwerveSubsystem swerveSubsystem, PoseEstimatior poseEstimatior) {
    m_SwerveSubsystem = swerveSubsystem;
    m_PoseEstimatior = poseEstimatior;

    m_AutoFactory =
        new AutoFactory(
            m_PoseEstimatior::get2dPose, // A function that returns the current robot pose
            m_PoseEstimatior::resetpose, // A function that resets the current robot pose to the provided Pose2d
            swerveSubsystem::followTrajectory,
            true, // If alliance flipping should be enabled
            m_SwerveSubsystem // The drive subsystem
            );
  }

  public AutoRoutine pickupAndScoreAuto() {
    AutoRoutine routine = m_AutoFactory.newRoutine("taxi");

    // Load the routine's trajectories
    AutoTrajectory driveToMiddle = routine.trajectory("driveToMiddle");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));
    return routine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
