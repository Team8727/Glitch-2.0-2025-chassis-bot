// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private PhotonCamera camera = new PhotonCamera("photonvision");
  /** Creates a new Vision. */
  public Vision() {

  }

  @Override
  public void periodic() {
    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 7) {
            // Found Tag 7, record its information
            targetYaw = target.getYaw();
            targetVisible = true;
          }
        }
      }
    }

    // TODO: fix this
    // // Auto-align when requested
    //     if (controller.getAButton() && targetVisible) {
    //         // Driver wants auto-alignment to tag 7
    //         // And, tag 7 is in sight, so we can turn toward it.
    //         // Override the driver's turn command with an automatic one that turns toward the tag.
    //         turn = -1.0 * targetYaw * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
    //     }

    //     // Command drivetrain motors based on target speeds
    //     drivetrain.drive(forward, strafe, turn);

    //     // Put debug information to the dashboard
    //     SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
    }  }
}
