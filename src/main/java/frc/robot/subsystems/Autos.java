// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.LinkedHashMap;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class Autos extends SubsystemBase {
 private final LEDSubsystem m_ledSubsytem;
private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<String, PathPlannerPath>();
  /** Creates a new Autos. */
  public Autos(LEDSubsystem ledSubsystem) {
   m_ledSubsytem = ledSubsystem;

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

  private void loadPaths() {
    try {
    paths.put("M-L4-H", PathPlannerPath.fromChoreoTrajectory("M-L4-H"));
    paths.put("H-PC", PathPlannerPath.fromChoreoTrajectory("H-PC"));
    paths.put("H-Refill", PathPlannerPath.fromChoreoTrajectory("H-Refill"));
    paths.put("CL-L4-I", PathPlannerPath.fromChoreoTrajectory("CL-L4-I"));
    paths.put("I-Refill", PathPlannerPath.fromChoreoTrajectory("I-Refill"));
    paths.put("Refill-J", PathPlannerPath.fromChoreoTrajectory("Refill-J"));
    paths.put("CR-L4-F", PathPlannerPath.fromChoreoTrajectory("CR-L4-F"));
    paths.put("F-Refill", PathPlannerPath.fromChoreoTrajectory("F-Refill"));
    paths.put("Refill-E", PathPlannerPath.fromChoreoTrajectory("Refill-E"));
    paths.put("M-L4-G", PathPlannerPath.fromChoreoTrajectory("M-L4-G"));
    paths.put("G-Refill", PathPlannerPath.fromChoreoTrajectory("G-Refill"));
    } catch (IOException e) {
      e.printStackTrace();
    } catch (ParseException e) {
      e.printStackTrace();
    }
  }




  public Command align(Pose2d goal) {
    return AutoBuilder.pathfindToPose(
        goal,
        new PathConstraints(
            kSwerve.Auton.maxOnTheFlyVel,
            kSwerve.Auton.maxOnTheFlyAcc,
            kSwerve.Auton.maxAngVel,
            kSwerve.Auton.maxAngAccel))
              .andThen(
                 run(() -> m_ledSubsytem.setPatternForDuration(
                    m_ledSubsytem.rainbow, 
                    2)))
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
              .andThen(
                run(
                  () -> m_ledSubsytem.setPatternForDuration(
                    m_ledSubsytem.rainbow, 
                    2)))
                    ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
