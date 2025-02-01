// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autos extends SubsystemBase {
  /** Creates a new Autos. */

  SwerveSubsystem swerveSubsystem;
  PoseEstimatior poseEstimatior;
  PhantomIntakePivot phantomIntakePivot;

  AutoFactory autoFactory;

  public Autos(SwerveSubsystem swerveSubsystem, PoseEstimatior poseEstimatior, PhantomIntakePivot phantomIntakePivot) {

    autoFactory = new AutoFactory(
            poseEstimatior::get2dPose, // A function that returns the current robot pose
            poseEstimatior::resetPoseToValue, // A function that resets the current robot pose to the provided Pose2d
            poseEstimatior::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            swerveSubsystem // The drive subsystem
          ); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
