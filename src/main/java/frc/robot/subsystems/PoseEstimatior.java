// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
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

public class PoseEstimatior extends SubsystemBase {
  private SwerveSubsystem m_SwerveSubsystem;

  /** Creates a new PoseEstimation. */

  public PoseEstimatior(
      SwerveSubsystem swerveSubsystem) {
    //subsystem setups
    m_SwerveSubsystem = swerveSubsystem;

  };

  //setup cameras 
  PhotonCamera camera1 = new PhotonCamera("camera1");
  PhotonCamera camera2 = new PhotonCamera("camera2");
  PhotonCamera camera3 = new PhotonCamera("camera3");
  PhotonCamera camera4 = new PhotonCamera("camera4");
  // photon pose estimators
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
  
  // get starting pos with cam1
  private Pose3d getPose3d() {
    //vars
    Pose3d pose3d = new Pose3d();
    var result = camera1.getLatestResult();
    boolean hasTargets = result.hasTargets();
    // if camera sees targets
    if (hasTargets) {
      //find best target
      PhotonTrackedTarget target = result.getBestTarget();
      if (kVision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        //estimate field to robot pose
        pose3d = PhotonUtils.estimateFieldToRobotAprilTag(
          target.getBestCameraToTarget(),
          kVision.aprilTagFieldLayout.getTagPose(
            target.getFiducialId()).get(),
             kVision.camera1Position);
      }
    }
    return pose3d;
  }

  SwerveDrivePoseEstimator3d swervePoseEstimator = new SwerveDrivePoseEstimator3d(
    kSwerve.kinematics,
    m_SwerveSubsystem.navX.getRotation3d(),
    m_SwerveSubsystem.modulePositions, 
    getPose3d());

  Optional<EstimatedRobotPose> getEstimatedGlobalPose(
      Pose3d prevEstimatedRobotPose,
      PhotonPipelineResult cameraResult,
      PhotonPoseEstimator PoseEstimator) {
    PoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return PoseEstimator.update(cameraResult);
  }
  @Override
  public void periodic() {
    //camera 1 pose estimation
    PhotonPipelineResult camera1res = camera1.getLatestResult();
    Optional<EstimatedRobotPose> camera1pose = getEstimatedGlobalPose(
      swervePoseEstimator.getEstimatedPosition(),
      camera1res,
      PoseEstimator1);
    swervePoseEstimator.addVisionMeasurement(
      camera1pose.get().estimatedPose,
      camera1pose.get().timestampSeconds);

    //camera 2 pose estimation
    PhotonPipelineResult camera2res = camera2.getLatestResult();
    Optional<EstimatedRobotPose> camera2pose = getEstimatedGlobalPose(
      swervePoseEstimator.getEstimatedPosition(),
      camera2res,
      PoseEstimator2);
    swervePoseEstimator.addVisionMeasurement(
      camera2pose.get().estimatedPose,
      camera2pose.get().timestampSeconds);

    //camera 3 pose estimation
    PhotonPipelineResult camera3res = camera3.getLatestResult();
    Optional<EstimatedRobotPose> camera3pose = getEstimatedGlobalPose(
      swervePoseEstimator.getEstimatedPosition(),
      camera3res,
      PoseEstimator3);
    swervePoseEstimator.addVisionMeasurement(
      camera3pose.get().estimatedPose,
      camera3pose.get().timestampSeconds);
    
    //camera 4 pose estimation
    PhotonPipelineResult camera4res = camera4.getLatestResult();
    Optional<EstimatedRobotPose> camera4pose = getEstimatedGlobalPose(
      swervePoseEstimator.getEstimatedPosition(),
      camera4res,
      PoseEstimator4);
    swervePoseEstimator.addVisionMeasurement(
      camera4pose.get().estimatedPose,
      camera4pose.get().timestampSeconds);

    //update pose
    swervePoseEstimator.update(m_SwerveSubsystem.navX.getRotation3d(), m_SwerveSubsystem.modulePositions);// may not need
  }
  
}
