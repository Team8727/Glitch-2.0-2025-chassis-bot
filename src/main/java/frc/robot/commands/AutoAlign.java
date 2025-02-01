// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.PoseEstimatior;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  SwerveSubsystem m_SwerveSubsystem;
  PoseEstimatior m_PoseEstimatior;

  double turningSpeed = 0;
  double botHeading;
  
  /** Creates a new AutoAlign. */
  public AutoAlign(SwerveSubsystem swerveSubsystem, PoseEstimatior poseEstimatior) {
    m_SwerveSubsystem = swerveSubsystem;
    m_PoseEstimatior = poseEstimatior;
    addRequirements(swerveSubsystem);
  }

  public Command align(Pose2d goal) {
    return 
      AutoBuilder.pathfindToPose(
        goal, 
        new PathConstraints(
          kSwerve.Auton.maxOnTheFlyVel,
          kSwerve.Auton.maxOnTheFlyAcc, 
          kSwerve.Auton.maxAngVel, 
          kSwerve.Auton.maxAngAccel));
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
