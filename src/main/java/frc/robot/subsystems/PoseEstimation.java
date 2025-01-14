// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;

public class PoseEstimation extends SubsystemBase {
  private final SwerveSubsystem m_SwerveSubsystem;
  private final Vision m_Vision;

  /** Creates a new PoseEstimation. */

  public PoseEstimation(SwerveSubsystem swerveSubsystem, Vision vision) {
    m_SwerveSubsystem = swerveSubsystem;
    m_Vision = vision;
    Pose3d pose3d = new Pose3d();
    var result = m_Vision.camera1.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets = true) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (kVision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        pose3d = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), kVision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), kVision.camera1Position);
      }
    }

    SwerveDrivePoseEstimator3d swervePoseEstimator = new SwerveDrivePoseEstimator3d(
      kSwerve.kinematics,
      m_SwerveSubsystem.navX.getRotation3d(),
      m_SwerveSubsystem.modulePositions, 
      pose3d);
  
  };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
