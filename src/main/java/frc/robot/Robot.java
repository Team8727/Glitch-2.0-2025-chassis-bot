// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PathFollowingController;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Kconfigs;
import frc.robot.Constants.kSwerve;
import frc.robot.commands.AutoAlignCmd;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.ChoreoPathCmd;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseEstimatior;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private final RobotContainer m_robotContainer;

  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final LEDSubsystem m_ledSubsytem = new LEDSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final PoseEstimatior m_PoseEstimatior = new PoseEstimatior(m_SwerveSubsystem);
  private final ChoreoPathCmd m_choreoPath  = new ChoreoPathCmd(m_SwerveSubsystem, m_PoseEstimatior);
  private final AutoAlignCmd m_AutoAlign = new AutoAlignCmd(m_SwerveSubsystem, m_PoseEstimatior);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    AutoFactory m_AutoFactory = new AutoFactory(
      m_PoseEstimatior::get2dPose, // A function that returns the current robot pose
      m_PoseEstimatior::resetpose, // A function that resets the current robot pose to the provided Pose2d
      m_SwerveSubsystem::followTrajectory,
      true, // If alliance flipping should be enabled 
      m_SwerveSubsystem // The drive subsystem
    );

    AutoBuilder.configure(
      m_PoseEstimatior::get2dPose,
      m_PoseEstimatior::resetpose,
      m_SwerveSubsystem::getChassisSpeeds, 
      (ChassisSpeeds, driveff) -> {
        System.out.println("aligning");
        // m_SwerveSubsystem.setModuleStates(
        //   kSwerve.kinematics.toSwerveModuleStates(speeds)),
        new DriveCmd(
          m_SwerveSubsystem,
          ChassisSpeeds,
          () -> true);
        },
      (PathFollowingController) kSwerve.Auton.pathFollowController,
      Kconfigs.robotConfig, 
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      m_SwerveSubsystem,
      m_PoseEstimatior);

    m_robotContainer = new RobotContainer(
      m_SwerveSubsystem, 
      m_ledSubsytem, 
      m_driverController, 
      m_PoseEstimatior,
      m_AutoAlign);

    PathfindingCommand.warmupCommand().schedule();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
