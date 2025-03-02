// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot.IntakePosition;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.commands.AlgaeIntake.IntakeAlgaeCmd;
// import frc.robot.commands.AlgaeIntake.ScoreAlgaeProcessorCmd;
import frc.robot.commands.Coral.DeployCoralCmd;
import frc.robot.commands.Coral.IntakeCoralCmd;
import frc.robot.commands.DriveCommands.SwerveJoystickCmd;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.Coral;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem;
  private final AlgaeIntakePivot m_AlgaeIntakePivot;
  private final AlgaeIntakeRollers m_AlgaeIntakeRollers;
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot;
  private final AlgaeRemoverRollers m_AlgaeRemoverRollers;
  private final Coral m_coral;
  private final Elevator m_elevator;
  private final CommandXboxController m_driverController;
  private final LEDSubsystem m_ledSubsytem;
  private final Autos m_Autos;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(
      SwerveSubsystem swerveSubsystem,
      AlgaeIntakePivot AlgaeIntakePivot,
      AlgaeIntakeRollers AlgaeIntakeRollers,
      AlgaeRemoverPivot AlgaeRemoverPivot,
      AlgaeRemoverRollers AlgaeRemoverRollers,
      Coral coral,
      Elevator elevator,
      CommandXboxController driverController,
      LEDSubsystem ledSubsystem,
      Autos autos
      ) {
    m_SwerveSubsystem = swerveSubsystem;
    m_AlgaeIntakePivot = AlgaeIntakePivot;
    m_AlgaeIntakeRollers = AlgaeIntakeRollers;
    m_AlgaeRemoverPivot = AlgaeRemoverPivot;
    m_AlgaeRemoverRollers = AlgaeRemoverRollers;
    m_coral = coral;
    m_elevator = elevator;
    m_driverController = driverController;
    m_ledSubsytem = ledSubsystem;
    m_Autos = autos;

    // Configure the trigger bindings
    configureBindings();
  }

  public void initiateJoystickOperated() {
    m_SwerveSubsystem.setDefaultCommand(
        new SwerveJoystickCmd(
            m_SwerveSubsystem,
            m_elevator,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> true,
            () -> true));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    /* To Test ChassisSpeeds */
    // m_driverController.a()
    //   .toggleOnTrue(new RunCommand(
    //     () -> 
    //       m_SwerveSubsystem.setModuleStates(
    //         kSwerve.kinematics.toSwerveModuleStates(
    //           new ChassisSpeeds(1,1,1)))));

    // Zero heading
    m_driverController.start().onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroHeading()));

    // // Remove algae L3
    // m_driverController.povUp().onTrue(new removeAlgae(m_AlgaeRemoverPivot, m_AlgeaRemoverRollers, 3, m_elevator));
    // // Remove algae L2
    // m_driverController.povDown().onTrue(new removeAlgae(m_AlgaeRemoverPivot, m_AlgeaRemoverRollers, 2, m_elevator));

    // intake coral
    m_driverController.leftBumper().onTrue(new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
    // Deploy coral L1
    m_driverController.x().onTrue(new DeployCoralCmd(m_coral, ElevatorPosition.L1, m_elevator, m_ledSubsytem));
    // Deploy coral L2
    m_driverController.y().onTrue(new DeployCoralCmd(m_coral, ElevatorPosition.L2, m_elevator, m_ledSubsytem));
    // Deploy coral L3
    m_driverController.b().onTrue(new DeployCoralCmd(m_coral, ElevatorPosition.L3, m_elevator, m_ledSubsytem));
    // Deploy coral L4
    m_driverController.a().onTrue(new DeployCoralCmd(m_coral, ElevatorPosition.L4, m_elevator, m_ledSubsytem));

    m_driverController.povDown().onTrue(new InstantCommand(() -> m_elevator.resetElevatorEncoders()));
    // Align to pose
    // m_driverController.povLeft().onTrue(m_Autos.align(kPoses.blueFrontLeft).andThen(() -> System.out.println("aligginginsdaod")));

    m_driverController.povRight().onTrue(new IntakeAlgaeCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers, m_ledSubsytem));

    m_driverController.povLeft().onTrue(new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));

    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_AlgaeIntakePivot.setPositionTrapazoidal(IntakePosition.HOME)));
    // m_driverController.rightBumper().onTrue(new ScoreAlgaeProcessorCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers, m_ledSubsytem));

    // // Intake algae
    // m_driverController.rightBumper().onTrue(
    //   new IntakeAlgaeCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers)
    //   .andThen(() -> 
    //     m_ledSubsytem.setPatternForDuration(
    //       m_ledSubsytem.solidRed, 1.5)
    //   ));

    // // Place algae in processor
    // m_driverController.rightTrigger().onTrue(
    //   new ScoreAlgaeProcessorCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers)
    //   .andThen(() -> 
    //     m_ledSubsytem.setPatternForDuration(
    //       m_ledSubsytem.solidGreen, 1.5)
    //   ));

    // X configuration
    // m_driverController.x().toggleOnTrue(m_SwerveSubsystem.XPosition());
    // Align to pose
    // m_driverController.b().onTrue(m_Autos.align(kPoses.blueFrontLeft));
    // Xbox Controller Bindings for LED Patterns
    // m_driverController.y().onTrue(new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.rainbow), m_ledSubsytem));
    // m_driverController.b().onTrue(new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.blue), m_ledSubsytem));

    // m_driverController.leftBumper()
    //     .onTrue(
    //         new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.red), m_ledSubsytem));
    // // m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.green), m_ledSubsytem));
    // m_driverController.a()
    //     .onTrue(
    //         new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.ace), m_ledSubsytem));
    // m_driverController.y()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_ledSubsytem.setPattern(m_ledSubsytem.colorCheck), m_ledSubsytem));

    // .until(() ->
    //     m_PoseEstimatior.get2dPose()
    //       .getTranslation()
    //     .getDistance(
    //       kPoses.blueFrontLeft.getTranslation())
    //     < 0.1
    //   &&
    //     m_PoseEstimatior.get2dPose()
    //       .getRotation()
    //     .minus(
    //       kPoses.blueFrontLeft.getRotation())
    //         .getDegrees()
    //     < 0.1)
    // .andThen(() -> joystickOperated())
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * Call this method from the {@link Robot#autonomousInit} method in order to run the autonomous command.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; //m_Autos.alignToPath(PathPlannerPath.fromChoreoTrajectory("ML-L4-FE"));
  }
}
