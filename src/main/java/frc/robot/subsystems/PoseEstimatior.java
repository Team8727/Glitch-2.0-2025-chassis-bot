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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;
import frc.robot.utilities.NetworkTableLogger;

public class PoseEstimatior extends SubsystemBase {
  SwerveSubsystem m_SwerveSubsystem;
  SwerveDrivePoseEstimator3d m_swervePoseEstimator;

  /** Creates a new PoseEstimation. */
  public PoseEstimatior(
      SwerveSubsystem swerveSubsystem) {
    // //setup simulation
    // VisionSystemSim visionSim = new VisionSystemSim("vision sim");
    // // Create the vision system simulation which handles cameras and targets on the field.
    // visionSim.addAprilTags(kVision.aprilTagFieldLayout);

    // // config simulation properties
    // SimCameraProperties cameraProp = new SimCameraProperties();
    // cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    // cameraProp.setCalibError(0.35, 0.10);
    // cameraProp.setFPS(15);
    // cameraProp.setAvgLatencyMs(50);
    // cameraProp.setLatencyStdDevMs(15);

    // PhotonCameraSim cameraSim = new PhotonCameraSim(camera1, cameraProp);
    
    // visionSim.addCamera(cameraSim, kVision.camera1Position);

    // cameraSim.enableDrawWireframe(true);

    //subsystem setups
    m_SwerveSubsystem = swerveSubsystem;
    m_swervePoseEstimator = swerveSubsystem.swervePoseEstimator;
    m_swervePoseEstimator.resetPose(getPose3d());
  };

  NetworkTableLogger networkTableLogger = new NetworkTableLogger("PoseEstimator");

  //setup cameras 
  PhotonCamera camera1 = new PhotonCamera("camera1");
  PhotonCamera camera2 = new PhotonCamera("camera2");
  PhotonCamera camera3 = new PhotonCamera("camera3");
  PhotonCamera camera4 = new PhotonCamera("camera4");  

  //Field2d for logging the robot's 2d position on the field to the dashboard like AdvantageScope, Elastic or Glass.
  private Field2d field2d = new Field2d();

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
  public Pose3d getPose3d() {
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

  // Get 2d pose: from the poseEstimator
  public Pose2d get2dPose() {
    return getPose3d().toPose2d();
  }

  public Field2d getField2d() {
    return field2d;
  }

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
      m_swervePoseEstimator.getEstimatedPosition(),
      camera1res,
      PoseEstimator1);
    try {
      m_swervePoseEstimator.addVisionMeasurement(
        camera1pose.get().estimatedPose,
        camera1pose.get().timestampSeconds);
        // System.out.println("not error");
      } catch (Exception e) {
        // System.out.println("error");
      }

    //camera 2 pose estimation
    PhotonPipelineResult camera2res = camera2.getLatestResult();
    Optional<EstimatedRobotPose> camera2pose = getEstimatedGlobalPose(
      m_swervePoseEstimator.getEstimatedPosition(),
      camera2res,
      PoseEstimator2);
    try {
      m_swervePoseEstimator.addVisionMeasurement(
        camera2pose.get().estimatedPose,
        camera2pose.get().timestampSeconds);
      } catch (Exception e) {
      }

    //camera 3 pose estimation
    PhotonPipelineResult camera3res = camera3.getLatestResult();
    Optional<EstimatedRobotPose> camera3pose = getEstimatedGlobalPose(
      m_swervePoseEstimator.getEstimatedPosition(),
      camera3res,
      PoseEstimator3);
    try {
      m_swervePoseEstimator.addVisionMeasurement(
        camera3pose.get().estimatedPose,
        camera3pose.get().timestampSeconds);
      } catch (Exception e) {
      }
    
    //camera 4 pose estimation
    PhotonPipelineResult camera4res = camera4.getLatestResult();
    Optional<EstimatedRobotPose> camera4pose = getEstimatedGlobalPose(
      m_swervePoseEstimator.getEstimatedPosition(),
      camera4res,
      PoseEstimator4);
    try {
      m_swervePoseEstimator.addVisionMeasurement(
        camera4pose.get().estimatedPose,
        camera4pose.get().timestampSeconds);
      } catch (Exception e) {
      }

    networkTableLogger.logDouble("null", 8);
        //Update Field2d with pose to display the robot's visual position on the field to the dashboard
    field2d.setRobotPose(get2dPose());

    //Log the robot's 2d position on the field to the dashboard using the NetworkTableLogger Utility
    networkTableLogger.log("Field2d", field2d);
    }
  // // simulation
  // @Override
  // public void simulationPeriodic() {
  //     // Update drivetrain simulation
  //     m_SwerveSubsystem.simulationPeriodic();

  //     // Update camera simulation
  //     simulationPeriodic(m_SwerveSubsystem.getHeading());

  //     var debugField = getSimDebugField();
  //     debugField.getObject("EstimatedRobot").setPose(m_SwerveSubsystem.getPose());
  //     debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());

  //     // Update gamepiece launcher simulation
  //     gpLauncher.simulationPeriodic();

  //     // Calculate battery voltage sag due to current draw
  //     RoboRioSim.setVInVoltage(
  //             BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw()));
  // }

}
