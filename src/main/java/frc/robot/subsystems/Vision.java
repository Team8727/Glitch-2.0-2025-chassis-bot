// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;

public class Vision extends SubsystemBase {

  public final PhotonCamera camera1 = new PhotonCamera("camera1");
  public final PhotonCamera camera2 = new PhotonCamera("camera2");
  public final PhotonCamera camera3 = new PhotonCamera("camera3");
  public static final PhotonCamera camera4 = new PhotonCamera("camera4");

  private PhotonPoseEstimator PoseEstimator1 = new PhotonPoseEstimator(
    kVision.aprilTagFieldLayout, 
    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
    kVision.camera1Position);
  private PhotonPoseEstimator PoseEstimator2 = new PhotonPoseEstimator(
    kVision.aprilTagFieldLayout, 
    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
    kVision.camera2Position);
  private PhotonPoseEstimator PoseEstimator3 = new PhotonPoseEstimator(
    kVision.aprilTagFieldLayout, 
    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
    kVision.camera3Position);
  private PhotonPoseEstimator PoseEstimator4 = new PhotonPoseEstimator(
    kVision.aprilTagFieldLayout, 
    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
    kVision.camera4Position);

  /** Creates a new Vision. */
  public Vision() {
  }


  @Override
  public void periodic() {

  } 
}
