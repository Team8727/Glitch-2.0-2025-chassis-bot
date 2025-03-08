// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.NoSuchElementException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.commands.Coral.DeployCoralCmd;
import frc.robot.commands.Coral.IntakeCoralCmd;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.commands.SetElevatorHeightCmd;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;

public class Autos extends SubsystemBase {
  private final LEDSubsystem m_ledSubsytem;
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final AlgaeIntakePivot m_algaeIntakePivot;
  private final AlgaeIntakeRollers m_algaeIntakeRollers;
  private final LinkedHashMap<String, PathPlannerPath> paths = new LinkedHashMap<String, PathPlannerPath>();
  private final PoseEstimatior m_PoseEstimatior;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final NetworkTableLogger logger = new NetworkTableLogger(this.getName().toString());
  private PathPlannerPath path1;


  /** Creates a new Autos. */
  public Autos(LEDSubsystem ledSubsystem, Coral coralSubsystem, Elevator elevatorSubsystem, AlgaeIntakePivot algaeIntakePivot, AlgaeIntakeRollers algaeIntakeRollers, PoseEstimatior poseEstimatior) {
    m_ledSubsytem = ledSubsystem;
    m_coral = coralSubsystem;
    m_elevator = elevatorSubsystem;
    m_algaeIntakePivot = algaeIntakePivot;
    m_algaeIntakeRollers = algaeIntakeRollers;
    m_PoseEstimatior = poseEstimatior;

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
    path1 = paths.get("H-PC");
  }

  private void loadPaths() {
    try {
    paths.put("M-L4-H", PathPlannerPath.fromChoreoTrajectory("M-L4-H"));
    paths.put("Red-M-L4-H", PathPlannerPath.fromChoreoTrajectory("M-L4-H").flipPath());
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
    paths.put("betterMinimum", PathPlannerPath.fromPathFile("bareMinimum"));
    loadPath("E-Refill");
    loadPath("ML-L4-I");
    loadPath("MR-L4-F");
    loadPath("bareminimum");
    loadPath("MR-L4-F");
    loadPath("ML-L4-I");
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  public void setupAutoChooser() {
    autoChooser.setDefaultOption("Path M-L4-H", M_L4_H());
    autoChooser.addOption("Path Red-M-L4-H", Red_M_L4_H());
    autoChooser.addOption("Path L-L4-I", L_L4_I());
    autoChooser.addOption("Path R_L4_I", R_L4_F());
    autoChooser.addOption("Path ML_L4_H", ML_L4_H());
    autoChooser.addOption("Path MR_L4_F", MR_L4_F());
    autoChooser.addOption("Path CL-L4-I-J", path_CL_L4_I_J());
    autoChooser.addOption("Path CR-L4-F-E", path_CR_L4_F_E());
    autoChooser.addOption("betterMinimum", bareMinimum());
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
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

  private void setStartPose(PathPlannerPath path) {
    Pose2d startPose = path.getStartingHolonomicPose().orElse(path.getStartingDifferentialPose());
    // if (DriverStation.getAlliance().get() ==  Alliance.Red) {
    //   Pose2d startPoseOffset = new Pose2d(
    //     (17.55 - startPose.getX()) + 0.01,
    //     (8 - startPose.getY()) + 0.01,
    //     new Rotation2d(
    //       Math.toRadians(
    //         startPose.getRotation().getDegrees() - 180)));
    //   m_PoseEstimatior.resetPoseToPose2d(startPoseOffset);
    //   path1 = path.flipPath();
    // } else {
      Pose2d startPoseOffset = new Pose2d(
        startPose.getX() - 0.01,
        startPose.getY() - 0.01,
        startPose.getRotation());
        m_PoseEstimatior.resetPoseToPose2d(startPoseOffset);
        // path1 = path;
    // }
  }

  private Command bareMinimum() {
    return alignToPath(paths.get("betterMinimum"));
  }

  private Command M_L4_H() {
    return new InstantCommand(() ->
      setStartPose(paths.get("M-L4-H")))
      .andThen(alignToPath(paths.get("M-L4-H")))
      .andThen(new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem)).withTimeout(3)
      .andThen(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator))
    ;
  }

  private Command Red_M_L4_H() {
    return new InstantCommand(() ->
      setStartPose(paths.get("Red-M-L4-H")))
      .andThen(alignToPath(paths.get("Red-M-L4-H")))
      .andThen(new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem)).withTimeout(3)
      .andThen(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator))
    ;
  }

  private Command L_L4_I() {
    return new InstantCommand(() -> 
      m_PoseEstimatior.resetPoseToPose2d(new Pose2d(7.2, 7.6, new Rotation2d(Math.toRadians(180)))))
      .andThen(alignToPath(paths.get("CL-L4-I"))
      .andThen(new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem)).withTimeout(2)
      .andThen(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator)));
  }

  private Command R_L4_F() {
    return new InstantCommand(() -> m_PoseEstimatior.resetPoseToPose2d(new Pose2d(7.2, 0.5, new Rotation2d(Math.toRadians(180)))))
      .andThen(alignToPath(paths.get("CR-L4-F")))
      .andThen(new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem)).withTimeout(2)
      .andThen(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator));
  }

  private Command MR_L4_F() {
    return new InstantCommand(() -> 
      m_PoseEstimatior.resetPoseToPose2d(new Pose2d(7.2, 3.65, new Rotation2d(Math.toRadians(180)))))
      .andThen(alignToPath(paths.get("MR-L4-F"))//TODO: add correct path
      .andThen(new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem)).withTimeout(2)
      .andThen(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator)));
  }

  private Command ML_L4_H() {
    return new InstantCommand(() -> 
      m_PoseEstimatior.resetPoseToPose2d(new Pose2d(7.2, 4.45, new Rotation2d(Math.toRadians(180)))))
      .andThen(alignToPath(paths.get("ML-L4-I"))//TODO: add correct path
      .andThen(new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem)).withTimeout(2)
      .andThen(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator)));
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
