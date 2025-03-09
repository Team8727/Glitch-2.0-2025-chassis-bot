// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimatior extends SubsystemBase {
  final SwerveSubsystem m_SwerveSubsystem;
  final SwerveDrivePoseEstimator3d m_SwervePoseEstimator;
  final NetworkTableLogger networkTableLogger = new NetworkTableLogger(this.getName().toString());

  PhotonCamera camera1 = new PhotonCamera("backRight");
  PhotonCamera camera2 = new PhotonCamera("backLeft");
  PhotonCamera camera3 = new PhotonCamera("front");
  PhotonCamera camera4 = new PhotonCamera("backUp");

  VisionSystemSim visionSim = new VisionSystemSim("main");

  PhotonCameraSim cameraSimBackRight;
  PhotonCameraSim cameraSimBackLeft;
  PhotonCameraSim cameraSimFront;
  PhotonCameraSim cameraSimBackUp;

  Pose3d latestCamPoseBackRiight = new Pose3d();
  Pose3d latestCamPoseBackLeft = new Pose3d();
  Pose3d latestCamPoseFront = new Pose3d();
  Pose3d latestCamPoseBackUp = new Pose3d();

  // TargetModel targetModel = TargetModel.kAprilTag16h5;
  // Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
  // VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

  SimCameraProperties cameraProp = new SimCameraProperties();

  // Field2d for logging the robot's 2d position on the field to the dashboard like AdvantageScope,
  // Elastic or Glass.
  public Field2d simField2d;
  public Field2d field2d = new Field2d();




  /** Creates a new PoseEstimation. */
  public PoseEstimatior(SwerveSubsystem swerveSubsystem) {
    // subsystem setups
    m_SwerveSubsystem = swerveSubsystem;
    m_SwervePoseEstimator = swerveSubsystem.SwervePoseEstimator;
    visionSim.addAprilTags(kVision.aprilTagFieldLayout);
    simField2d = visionSim.getDebugField();
    setUpCameras();
    resetStartPose();
  }

  private void setUpCameras() {
    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(40);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    cameraSimBackRight = new PhotonCameraSim(camera1, cameraProp);
    cameraSimBackLeft = new PhotonCameraSim(camera2, cameraProp);
    cameraSimFront = new PhotonCameraSim(camera3, cameraProp);
    cameraSimBackUp = new PhotonCameraSim(camera4, cameraProp);

    visionSim.addCamera(cameraSimBackRight, kVision.camera1Position);
    visionSim.addCamera(cameraSimBackLeft, kVision.camera2Position);
    visionSim.addCamera(cameraSimFront, kVision.camera4Position);
    visionSim.addCamera(cameraSimBackUp, kVision.camera3Position);
    }

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
          kVision.camera4Position);
  PhotonPoseEstimator PoseEstimator4 =
      new PhotonPoseEstimator(
          kVision.aprilTagFieldLayout,
          PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
          kVision.camera3Position);

  // get starting pos with cam1
  public Pose3d getPose3d() {
    // vars
    Pose3d pose3d = new Pose3d();
    PhotonPipelineResult result = camera1.getLatestResult();
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
  public void simulationPeriodic() {
    visionSim.update(m_SwervePoseEstimator.getEstimatedPosition());
    networkTableLogger.logPose3d("cam back Right", visionSim.getCameraPose(cameraSimBackRight).orElse(null));
    networkTableLogger.logPose3d("cam back left",  visionSim.getCameraPose(cameraSimBackLeft).orElse(null));
    networkTableLogger.logPose3d("cam front",     visionSim.getCameraPose(cameraSimFront).orElse(null));
    networkTableLogger.logPose3d("cam back up",   visionSim.getCameraPose(cameraSimBackUp).orElse(null));
    m_SwervePoseEstimator.addVisionMeasurement(visionSim.getRobotPose(), 0);
  }

  @Override
  public void periodic() {
    // // camera 1 pose estimation
    // addVisionMeasurement(camera1, PoseEstimator1);
    // // // camera 2 pose estimation
    // addVisionMeasurement(camera2, PoseEstimator2);
    // // // camera 3 pose estimation
    // addVisionMeasurement(camera3, PoseEstimator3);
    // // // camera 4 pose estimation
    // addVisionMeasurement(camera4, PoseEstimator4);

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
    networkTableLogger.logField2d("simField", simField2d);
    networkTableLogger.logPose2d("2d pose", get2dPose());
    networkTableLogger.logPose2d("Robot 3d Pose", get2dPose());
    networkTableLogger.logPose3d("Robot 2d Pose", m_SwervePoseEstimator.getEstimatedPosition());
  }
}
