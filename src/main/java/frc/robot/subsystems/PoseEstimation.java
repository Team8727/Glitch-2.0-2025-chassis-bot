// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;

public class PoseEstimation extends SubsystemBase {
  private SwerveSubsystem m_SwerveSubsystem;
  private Vision m_Vision;

  /** Creates a new PoseEstimation. */

  public PoseEstimation(SwerveSubsystem swerveSubsystem, Vision vision) {
    m_SwerveSubsystem = swerveSubsystem;
    m_Vision = vision;
  };

  PhotonPoseEstimator PoseEstimator1 = new PhotonPoseEstimator(
    kVision.aprilTagFieldLayout, 
    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
    kVision.camera1Position);
  PhotonPoseEstimator PoseEstimator2 = new PhotonPoseEstimator(
    kVision.aprilTagFieldLayout, 
    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
    kVision.camera2Position);
  PhotonPoseEstimator PoseEstimator3 = new PhotonPoseEstimator(
    kVision.aprilTagFieldLayout, 
    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
    kVision.camera3Position);
  PhotonPoseEstimator PoseEstimator4 = new PhotonPoseEstimator(
    kVision.aprilTagFieldLayout, 
    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
    kVision.camera4Position);
  
  private Pose3d getPose3d() {
    Pose3d pose3d = new Pose3d();
    var result = m_Vision.camera1.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (kVision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        pose3d = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), kVision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), kVision.camera1Position);
      }
    }
    return pose3d;
  }

  SwerveDrivePoseEstimator3d swervePoseEstimator = new SwerveDrivePoseEstimator3d(
    kSwerve.kinematics,
    m_SwerveSubsystem.navX.getRotation3d(),
    m_SwerveSubsystem.modulePositions, 
    getPose3d());

  Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose, PhotonPipelineResult cameraResult) {
    PoseEstimator1.setReferencePose(prevEstimatedRobotPose);
    return PoseEstimator1.update(cameraResult);
  }
  @Override
  public void periodic() {
    PhotonPipelineResult camera1res = m_Vision.camera1.getLatestResult();
    Optional<EstimatedRobotPose> camera1pose = getEstimatedGlobalPose(swervePoseEstimator.getEstimatedPosition(), camera1res);
    swervePoseEstimator.addVisionMeasurement(camera1pose.get().estimatedPose,camera1pose.get().timestampSeconds);

    // PoseEstimator1.setReferencePose();
    // return PoseEstimator1.update();
  }
    // This method will be called once per scheduler run
  
}
