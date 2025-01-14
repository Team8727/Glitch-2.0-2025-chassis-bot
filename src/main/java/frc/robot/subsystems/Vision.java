// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  public final PhotonCamera camera1 = new PhotonCamera("camera1");
  public final PhotonCamera camera2 = new PhotonCamera("camera2");
  public final PhotonCamera camera3 = new PhotonCamera("camera3");
  public final PhotonCamera camera4 = new PhotonCamera("camera4");

  /** Creates a new Vision. */
  public Vision() {
  }


  @Override
  public void periodic() {

  } 
}
