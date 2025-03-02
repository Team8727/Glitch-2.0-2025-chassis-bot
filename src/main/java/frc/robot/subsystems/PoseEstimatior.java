// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;
import frc.robot.utilities.NetworkTableLogger;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimatior extends SubsystemBase {
  SwerveSubsystem m_SwerveSubsystem;
  SwerveDrivePoseEstimator3d m_SwervePoseEstimator;
  NetworkTableLogger networkTableLogger = new NetworkTableLogger(this.getName().toString());

  /** Creates a new PoseEstimation. */
  public PoseEstimatior(SwerveSubsystem swerveSubsystem) {
    // subsystem setups
    m_SwerveSubsystem = swerveSubsystem;
    m_SwervePoseEstimator = swerveSubsystem.SwervePoseEstimator;
    resetStartPose();
  }

  // setup cameras
  PhotonCamera camera1 = new PhotonCamera("backRight");
  PhotonCamera camera2 = new PhotonCamera("backLeft");
  PhotonCamera camera3 = new PhotonCamera("front");
  PhotonCamera camera4 = new PhotonCamera("backUp");

  // Field2d for logging the robot's 2d position on the field to the dashboard like AdvantageScope,
  // Elastic or Glass.
  public Field2d field2d = new Field2d();

  // photon pose estimators
  PhotonPoseEstimator PoseEstimator1 =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.camera1Position);
  PhotonPoseEstimator PoseEstimator2 =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.camera2Position);
  PhotonPoseEstimator PoseEstimator3 =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.camera3Position);
  PhotonPoseEstimator PoseEstimator4 =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.camera4Position);

  // get starting pos with cam1
  public Pose3d getPose3d() {
    // vars
    Pose3d pose3d = new Pose3d();
    var result = camera1.getLatestResult();
    boolean hasTargets = result.hasTargets();
    // if camera sees targets
    if (hasTargets) {
      // find best target
      PhotonTrackedTarget target = result.getBestTarget();
      if (kVision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        // estimate field to robot pose
        pose3d =
            PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                kVision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                kVision.camera1Position);
      }
    }
    return pose3d;
  }

  public void resetPoseToPose3d(Pose3d pose3d) {
    m_SwervePoseEstimator.resetPose(pose3d);
  }

  public void resetPoseToPose2d(Pose2d pose2d) {
    Pose3d pose3d = new Pose3d(pose2d);
    m_SwervePoseEstimator.resetPose(pose3d);
  }

  public void resetStartPose() {
    m_SwervePoseEstimator.resetPose(getPose3d());
  }

  public void resetToEmptyPose() {
    Pose3d pose3d = new Pose3d();
    m_SwervePoseEstimator.resetPose(pose3d);
  }

  // Get 2d pose: from the poseEstimator
  public Pose2d get2dPose() {
    return (m_SwervePoseEstimator.getEstimatedPosition().toPose2d());
  }

  Optional<EstimatedRobotPose> getEstimatedGlobalPose(
      Pose3d prevEstimatedRobotPose,
      PhotonPipelineResult cameraResult,
      PhotonPoseEstimator PoseEstimator) {
    PoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return PoseEstimator.update(cameraResult);
  }

  private void addVisionMeasurement(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
    try {
      // camera 4 pose estimation
      List<PhotonPipelineResult> cameraRes = camera.getAllUnreadResults();
      PhotonPipelineResult cameraLatestRes = cameraRes.get(cameraRes.size() - 1);

      List<PhotonTrackedTarget> targets = cameraLatestRes.getTargets();
      for (PhotonTrackedTarget target : targets) {
        double ambiguity = target.getPoseAmbiguity();

        if (ambiguity <= 0.2) {
          Optional<EstimatedRobotPose> cameraPose =
            getEstimatedGlobalPose(
                m_SwervePoseEstimator.getEstimatedPosition(),
                cameraLatestRes, 
                poseEstimator);
        
          m_SwervePoseEstimator.addVisionMeasurement(
              cameraPose.get().estimatedPose, cameraPose.get().timestampSeconds);
        }
      }
    } catch (Exception e) {
    }
  }

  @Override
  public void periodic() {
    // // camera 1 pose estimation
    // addVisionMeasurement(camera1, PoseEstimator1);
    // // camera 2 pose estimation
    // addVisionMeasurement(camera2, PoseEstimator2);
    // // camera 3 pose estimation
    // addVisionMeasurement(camera3, PoseEstimator3);
    // // camera 4 pose estimation
    // addVisionMeasurement(camera4, PoseEstimator4);

    // try {
    //   // camera 1 pose estimation
    //   List<PhotonPipelineResult> camera1res = camera1.getAllUnreadResults();
    //   PhotonPipelineResult camera1LatestRes = camera1res.get(camera1res.size() - 1);
    //   Optional<EstimatedRobotPose> camera1pose =
    //     getEstimatedGlobalPose(
    //         m_SwervePoseEstimator.getEstimatedPosition(), camera1LatestRes, PoseEstimator1);
    //   m_SwervePoseEstimator.addVisionMeasurement(
    //       camera1pose.get().estimatedPose, camera1pose.get().timestampSeconds);
    //   // System.out.println("not error");
    // } catch (Exception e) {
    //   // System.out.println("error");
    // }

    // try {
    //   // camera 2 pose estimation
    //   List<PhotonPipelineResult> camera2res = camera2.getAllUnreadResults();
    //   PhotonPipelineResult camera2LatestRes = camera2res.get(camera2res.size() - 1);
    //   Optional<EstimatedRobotPose> camera2pose =
    //     getEstimatedGlobalPose(
    //         m_SwervePoseEstimator.getEstimatedPosition(), camera2LatestRes, PoseEstimator2);
    //   m_SwervePoseEstimator.addVisionMeasurement(
    //       camera2pose.get().estimatedPose, camera2pose.get().timestampSeconds);
    // } catch (Exception e) {
    // }

    // try {
    //   // camera 3 pose estimation
    //   List<PhotonPipelineResult> camera3res = camera3.getAllUnreadResults();
    //   PhotonPipelineResult camera3LatestRes = camera3res.get(camera3res.size() - 1);
    //   Optional<EstimatedRobotPose> camera3pose =
    //     getEstimatedGlobalPose(
    //         m_SwervePoseEstimator.getEstimatedPosition(), camera3LatestRes, PoseEstimator3);
    //   m_SwervePoseEstimator.addVisionMeasurement(
    //       camera3pose.get().estimatedPose, camera3pose.get().timestampSeconds);
    // } catch (Exception e) {
    // }

    // try {
    //   // camera 4 pose estimation
    //   List<PhotonPipelineResult> camera4res = camera4.getAllUnreadResults();
    //   PhotonPipelineResult camera4LatestRes = camera4res.get(camera4res.size() - 1);

    //   List<PhotonTrackedTarget> targets = camera4LatestRes.getTargets();
    //   for (PhotonTrackedTarget target : targets) {
    //     double ambiguity = target.getPoseAmbiguity();

    //     if (ambiguity <= 0.2) {
    //       Optional<EstimatedRobotPose> camera4pose =
    //         getEstimatedGlobalPose(
    //             m_SwervePoseEstimator.getEstimatedPosition(),
    //             camera4LatestRes, 
    //             PoseEstimator4);
        
    //       m_SwervePoseEstimator.addVisionMeasurement(
    //           camera4pose.get().estimatedPose, camera4pose.get().timestampSeconds);
    //     }
    //   }
    // } catch (Exception e) {
    // }

    // gyro update
    m_SwervePoseEstimator.update(
        m_SwerveSubsystem.navX.getRotation3d(), m_SwerveSubsystem.modulePositions);

    // Update Field2d with pose to display the robot's visual position on the field to the dashboard
    field2d.setRobotPose(get2dPose());

    // field.setRobotPose(m_swervePoseEstimator.getEstimatedPosition().toPose2d());//pose 3d as 2d
    // pose

    // Log the robot's 2d position on the field to the dashboard using the NetworkTableLogger
    // Utility
    networkTableLogger.logField2d("Field2d", field2d);
    networkTableLogger.logPose2d("2d pose", get2dPose());
    networkTableLogger.logPose2d("Robot 3d Pose", get2dPose());
    networkTableLogger.logPose3d("Robot 2d Pose", m_SwervePoseEstimator.getEstimatedPosition());
  }
}
