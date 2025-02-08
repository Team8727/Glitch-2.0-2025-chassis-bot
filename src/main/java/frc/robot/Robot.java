// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kConfigs;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseEstimatior;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.Coral;

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
  private final Autos m_Autos = new Autos();
  private final AlgaeRemoverRollers m_AlgeaRemoverRollers = new AlgaeRemoverRollers();
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot = new AlgaeRemoverPivot();
  private final Coral m_coral = new Coral();
  private final Elevator m_elevator = new Elevator();
  private final AlgaeIntakePivot m_AlgaeIntakePivot = new AlgaeIntakePivot();
  private final AlgaeIntakeRollers m_AlgaeIntakeRollers = new AlgaeIntakeRollers();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    AutoBuilder.configure(
        m_PoseEstimatior::get2dPose,
        m_PoseEstimatior::resetPoseToPose2d,
        m_SwerveSubsystem::getChassisSpeeds,
        (chassisSpeeds, driveff) -> {
          System.out.println("aligning");
          m_SwerveSubsystem.setModuleStates(kSwerve.kinematics.toSwerveModuleStates(chassisSpeeds));
          // new DriveCmd(m_SwerveSubsystem, () -> chassisSpeeds, () -> true).execute();
        },
        (PathFollowingController) kSwerve.Auton.pathFollowController,
        kConfigs.robotConfig,
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

    m_robotContainer =
        new RobotContainer(
            m_SwerveSubsystem,
            m_ledSubsytem,
            m_driverController,
            m_Autos,
            m_AlgaeRemoverPivot,
            m_AlgeaRemoverRollers,
            m_coral,
            m_elevator,
            m_AlgaeIntakePivot,
            m_AlgaeIntakeRollers);

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
    m_robotContainer.getAutonomousCommand();
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
    m_robotContainer.getAutonomousCommand().cancel();

    m_robotContainer.initiateJoystickOperated();
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

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}
