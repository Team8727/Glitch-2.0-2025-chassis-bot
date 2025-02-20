package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kElevator;
import frc.robot.Constants.kOI;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.function.Supplier;

public class SwerveJoystickCmd extends Command {

  private final SwerveSubsystem m_SwerveSubsystem;
  private final Elevator m_Elevator;
  private final Supplier<Double> m_xSpdFunction, m_ySpdFunction, m_turningSpdFunction;
  private final Supplier<Boolean> m_fieldOrientedFunction;
  private final Supplier<Boolean> m_scaleSpeedToElevHeight;

  public SwerveJoystickCmd(
      SwerveSubsystem swerveSubsystem,
      Elevator elevator,
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction,
      Supplier<Boolean> scaleSpeedToElevHeight) {
    m_SwerveSubsystem = swerveSubsystem;
    m_Elevator = elevator;
    m_xSpdFunction = xSpdFunction;
    m_ySpdFunction = ySpdFunction;
    m_turningSpdFunction = turningSpdFunction;
    m_fieldOrientedFunction = fieldOrientedFunction;
    m_scaleSpeedToElevHeight = scaleSpeedToElevHeight;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // get joystick values
    double xSpeed = m_xSpdFunction.get();
    double ySpeed = m_ySpdFunction.get();
    double turningSpeed = m_turningSpdFunction.get();

    // apply deadband
    xSpeed = MathUtil.applyDeadband(xSpeed, kOI.translationDeadzone);
    ySpeed = MathUtil.applyDeadband(ySpeed, kOI.translationDeadzone);
    turningSpeed = MathUtil.applyDeadband(turningSpeed, kOI.rotationDeadzone);

    if (m_scaleSpeedToElevHeight.get()) { // Scaling to elevator height

      // get elevator height for anti-tipping
      double elevatorHeight = m_Elevator.getElevatorHeight();
      xSpeed = xSpeed * kSwerve.maxTransSpeed * (elevatorHeight / kElevator.ElevatorPosition.L4.getOutputRotations())
       + kSwerve.DriveSpeedScaling.minDriveSpeed; // Scaling to elevator height
      ySpeed *= kSwerve.maxTransSpeed * (elevatorHeight / kElevator.ElevatorPosition.L4.getOutputRotations())
       + kSwerve.DriveSpeedScaling.minDriveSpeed; // Scaling to elevator height
      turningSpeed *= kSwerve.maxAngSpeed * (elevatorHeight / kElevator.ElevatorPosition.L4.getOutputRotations())
       + kSwerve.DriveSpeedScaling.minDriveSpeed; // Scaling to elevator height
    
    } else {
      xSpeed *= kSwerve.maxTransSpeed;
      ySpeed *= kSwerve.maxTransSpeed;
      turningSpeed *= kSwerve.maxAngSpeed;
    }

    // set chassis speed
    ChassisSpeeds chassisSpeeds;
    if (m_fieldOrientedFunction.get()) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, turningSpeed, m_SwerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // Set the swerve module states
    SwerveModuleState[] moduleStates = kSwerve.kinematics.toSwerveModuleStates(chassisSpeeds);
    // System.out.println("setting module states");

    // output to swerve modules
    m_SwerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    // m_SwerveSubsystem.stopModules(); // does nothing
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
