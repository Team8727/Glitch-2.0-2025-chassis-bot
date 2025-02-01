// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Kconfigs;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.PoseEstimatior;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
    AutoBuilder.configure(
      m_PoseEstimatior::get2dPose,
      (pose) -> m_PoseEstimatior.resetpose(),
      m_SwerveSubsystem::getChassisSpeeds, 
      (speeds) -> {
        System.out.println("aligning");
        // m_SwerveSubsystem.setModuleStates(
        //   kSwerve.kinematics.toSwerveModuleStates(speeds)),
        new SwerveJoystickCmd(
          m_SwerveSubsystem,
          () -> speeds.vxMetersPerSecond, 
          () -> speeds.vyMetersPerSecond, 
          () -> speeds.omegaRadiansPerSecond,
          () -> true);},
      (PathFollowingController) kSwerve.Auton.pathFollowController,
      Kconfigs.robotConfig, 
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      m_SwerveSubsystem
      );

      PathfindingCommand.warmupCommand().schedule();
  }

  
  public Command followPath(Pose2d goal) {
    return 
      new RunCommand( 
        () -> { 
        System.out.println("pathfind start");
        AutoBuilder.followPath(PathPlannerPath.waypointsFromPoses(m_PoseEstimatior.get2dPose(), goal));
        System.out.println("pathfind end");});
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
