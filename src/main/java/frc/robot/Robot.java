// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.utilities.NetworkTableLogger;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private final RobotContainer m_robotContainer;
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final PoseEstimatior m_PoseEstimatior = new PoseEstimatior(m_SwerveSubsystem);
  private final LEDSubsystem m_ledSubsytem = new LEDSubsystem();
  private final NetworkTableLogger logger = new NetworkTableLogger("SHOW UPPPP");
  private final AlgaeRemoverRollers m_AlgeaRemoverRollers = new AlgaeRemoverRollers();
  private final AlgaeRemoverPivot m_AlgaeRemoverPivot = new AlgaeRemoverPivot();
  private final Coral m_coral = new Coral();
  private final Elevator m_elevator = new Elevator();
  private final AlgaeIntakePivot m_AlgaeIntakePivot = new AlgaeIntakePivot();
  private final AlgaeIntakeRollers m_AlgaeIntakeRollers = new AlgaeIntakeRollers();
  private final Autos m_Autos = new Autos(m_ledSubsytem, m_coral, m_elevator);

  private boolean m_elevatorSpeedControl = true;

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
          logger.logChassisSpeeds("speeds", chassisSpeeds);
          // m_PoseEstimatior.field2d.getObject("Trajectory").setTrajectory(); //TODO: get this from pathplanner somehow
          m_SwerveSubsystem.setModuleStates(kSwerve.kinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, m_SwerveSubsystem.getRotation2d())));
          // new DriveCmd(m_SwerveSubsystem, () -> chassisSpeeds, () -> true).execute();
        },
        kSwerve.Auton.pathFollowController,
        kConfigs.robotConfig,
        () -> false,
        // () -> {
        //   // Boolean supplier that controls when the path will be mirrored for the red alliance
        //   // This will flip the path being followed to the red side of the field.
        //   // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        //   Optional<Alliance> alliance = DriverStation.getAlliance();

        //   if (alliance.isPresent()) {
        //     return alliance.get() == DriverStation.Alliance.Red;
        //   }
        //   return true;
        // },
        m_SwerveSubsystem,
        m_PoseEstimatior);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer =
        new RobotContainer(
            m_SwerveSubsystem,
            m_AlgaeIntakePivot,
            m_AlgaeIntakeRollers,
            m_AlgaeRemoverPivot,
            m_AlgeaRemoverRollers,
            m_coral,
            m_elevator,
            m_driverController,
            m_ledSubsytem,
            m_Autos,
            m_elevatorSpeedControl
            );

    // Set Up PathPlanner to "warm up" the pathplanning system
    PathfindingCommand.warmupCommand().schedule();

    // Log data to a log file using WPILib's DataLogManager
    DataLogManager.start();

    // Start the URCL logger (logs REV SparkMaxes and SparkFlexes automatically on networktables)
    URCL.start();

    // Start logging subsystem values //TODO: Uncomment when m_AlgaeIntakePivot and m_AlgaeIntakeRollers are implemented and on the robot
    // m_AlgaeIntakePivot.shouldLogValues(true);
    // m_AlgaeIntakeRollers.shouldLogValues(true);
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
    logger.logDouble("voltage", RobotController.getInputVoltage());

    // m_elevatorSpeedControl = logger.getBoolean("Limit Speed", true);

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
    // m_Autos.M_H(); // TODO: Only enable this if you want the robot to do stuff during autonomous
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
    
    // m_robotContainer.getAutonomousCommand().cancel();

    m_robotContainer.initiateJoystickOperated();
}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  // -=-=-=-=-=-=-=- SysIdRoutines for the AlgaeIntakePivot -=-=-=-=-=-=-=-=-
  
  /* TODO: Run these SysIdRoutines for the AlgaeIntakePivot using the button triggers and then 
     follow these instructions starting with "SysId Usage" step 3: 
     https://docs.advantagescope.org/more-features/urcl 
  */

    // // Quasistatic SysIdRoutines
    // m_driverController.povUp()
    // .and(m_driverController.leftTrigger())
    //   .onTrue(m_AlgaeIntakePivot.sysIdRoutine_quasistatic_fwd())
    //   .onFalse(m_AlgaeIntakePivot.stopSysIdRoutine());

    // m_driverController.povDown()
    // .and(m_driverController.leftTrigger())
    //   .onTrue(m_AlgaeIntakePivot.sysIdRoutine_quasistatic_rev())
    //   .onFalse(m_AlgaeIntakePivot.stopSysIdRoutine());

    // // Dynamic SysIdRoutines
    // m_driverController.y()
    // .and(m_driverController.leftTrigger())
    //   .onTrue(m_AlgaeIntakePivot.sysIdRoutine_dynamic_fwd())
    //   .onFalse(m_AlgaeIntakePivot.stopSysIdRoutine());

    // m_driverController.a()
    // .and(m_driverController.leftTrigger())
    //   .onTrue(m_AlgaeIntakePivot.sysIdRoutine_dynamic_fwd())
    //   .onFalse(m_AlgaeIntakePivot.stopSysIdRoutine());

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
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
