// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class Autos extends SubsystemBase {
//  private final LEDSubsystem m_ledSubsytem; DO NOT USE THE LED CODE UNTIL IT IS FIXED
  /** Creates a new Autos. */
  public Autos() {
  //  m_ledSubsytem = new LEDSubsystem();

  // // Load a full Choreo trajectory as a PathPlannerPath
  // PathPlannerPath exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj");
  // // Load a split Choreo trajectory as a PathPlannerPath, using the split point with index 1
  // PathPlannerPath exampleChoreoTrajSplit = PathPlannerPath.fromChoreoTrajectory("Example Choreo Traj", 1);

    /* Add paths to the hashmap using this format:
      paths.put("<Name>", PathPlannerPath.fromPathFile("<path file name>"));
      ex:
      paths.put("Crazy auto", swerve.followPathWithEventsCommand(paths.get("crazy_auto")));
    */
    
    //paths.put("EXAMPLE", PathPlannerPath.fromChoreoTrajectory("EXAMPLE"));

  }




  public Command align(Pose2d goal) {
    return AutoBuilder.pathfindToPose(
        goal,
        new PathConstraints(
            kSwerve.Auton.maxOnTheFlyVel,
            kSwerve.Auton.maxOnTheFlyAcc,
            kSwerve.Auton.maxAngVel,
            kSwerve.Auton.maxAngAccel))
              // .andThen(
              //    run(() -> m_ledSubsytem.setPatternForDuration(
              //       m_ledSubsytem.rainbow, 
              //       2)))
                    ;
  }

  public Command alignToPath(PathPlannerPath goal) {
    return AutoBuilder.pathfindThenFollowPath(
        goal,
        new PathConstraints(
            kSwerve.Auton.maxOnTheFlyVel,
            kSwerve.Auton.maxOnTheFlyAcc,
            kSwerve.Auton.maxAngVel,
            kSwerve.Auton.maxAngAccel))
              // .andThen(
              //   run(
              //     () -> m_ledSubsytem.setPatternForDuration(
              //       m_ledSubsytem.rainbow, 
              //       2)))
                    ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
