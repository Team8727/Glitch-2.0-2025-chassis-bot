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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.commands.AlgaeIntake.ScoreAlgaeCmd;
import frc.robot.commands.Coral.DeployCoralCmd;
import frc.robot.commands.Coral.IntakeCoralCmd;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.commands.SetElevatorHeightCmd;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;

public class Autos extends SubsystemBase {
  private final LEDSubsystem m_ledSubsytem;
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final AlgaeIntakePivot m_algaeIntakePivot;
  private final AlgaeIntakeRollers m_algaeIntakeRollers;
  private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<String, PathPlannerPath>();

  /** Creates a new Autos. */
  public Autos(LEDSubsystem ledSubsystem, Coral coralSubsystem, Elevator elevatorSubsystem, AlgaeIntakePivot algaeIntakePivot, AlgaeIntakeRollers algaeIntakeRollers) {
    m_ledSubsytem = ledSubsystem;
    m_coral = coralSubsystem;
    m_elevator = elevatorSubsystem;
    m_algaeIntakePivot = algaeIntakePivot;
    m_algaeIntakeRollers = algaeIntakeRollers;

    loadPaths();

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
    paths.put("J-Refill", PathPlannerPath.fromChoreoTrajectory("J-Refill"));
    loadPath("E-Refill");
    loadPath("ML-L4-I");
    loadPath("MR-L4-F");
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  private void loadPath(String pathName) {
    try {
    paths.put(pathName, PathPlannerPath.fromChoreoTrajectory(pathName));
    } catch (IOException | ParseException e) {
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
            kSwerve.Auton.maxAngAccel)).andThen(new WaitCommand(0.0001));
  }

  private Command path_M_L4_H() {
    return new SequentialCommandGroup(
      alignToPath(paths.get("M-L4-H")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem,m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      alignToPath(paths.get("H-Refill")),
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
  }

  public Command CR_FE() {
    return new SequentialCommandGroup(
      alignToPath(paths.get("CR-L4-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      alignToPath(paths.get("F-Refill")),
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem),
      alignToPath(paths.get("Refill-E")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem)
    );
  }

  public Command path_ML_L4_I_J() {
    return alignToPath(paths.get("M-L4-H"))
      .andThen(new SetElevatorHeightCmd(ElevatorPosition.L2, m_elevator, m_coral, m_ledSubsytem)).andThen(new WaitCommand(0.0001))
      .andThen(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator));
      // new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      // alignToPath(paths.get("I-Refill")),
      // new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
  }

  public Command ML_I_R_J_R() {
    return new SequentialCommandGroup(
      alignToPath(paths.get("ML-L4-I")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      alignToPath(paths.get("I-Refill")),
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem),
      alignToPath(paths.get("Refill-J")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      alignToPath(paths.get("J-Refill")),
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
  }

  public Command MR_F_R_E_R() {
    return new SequentialCommandGroup(
      alignToPath(paths.get("MR-L4-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      alignToPath(paths.get("F-Refill")),
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem),
      alignToPath(paths.get("Refill-E")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      alignToPath(paths.get("E-Refill")),
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
  }

  private Command path_CL_L4_I_J() {
    return new SequentialCommandGroup(
      alignToPath(paths.get("CL-L4-I")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      alignToPath(paths.get("I-Refill")),
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem),
      alignToPath(paths.get("Refill-J")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem));
  }

  private Command path_CR_L4_F_E() {
    return new SequentialCommandGroup(
      alignToPath(paths.get("CR-L4-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      alignToPath(paths.get("F-Refill")),
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem),
      alignToPath(paths.get("Refill-E")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem,m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem));
  }

  private Command path_MR_L4_F() {
    return new SequentialCommandGroup(
      alignToPath(paths.get("M-L4-F")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem),
      alignToPath(paths.get("F-Refill")),
      new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem),
      alignToPath(paths.get("Refill-E")),
      new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem),
      new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator),
      new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command getAutonomousCommand() { // TODO: This is where our autonomous commands will be run, check to see if it works
    // An example command will be run in autonomous
    return 
    null;
  }
}
