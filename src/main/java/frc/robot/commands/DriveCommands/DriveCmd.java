package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.NetworkTableLogger;

public class DriveCmd extends Command {

  private final SwerveSubsystem m_SwerveSubsystem;
  private final Boolean m_fieldOrientedFunction;
  private ChassisSpeeds m_chassisSpeeds;
  private SwerveModuleState[] moduleStates = kSwerve.kinematics.toSwerveModuleStates(new ChassisSpeeds());
  private final NetworkTableLogger logger = new NetworkTableLogger("modules");


  public DriveCmd(
      SwerveSubsystem swerveSubsystem,
      Boolean fieldOrientedFunction,
      ChassisSpeeds speeds) {
    m_SwerveSubsystem = swerveSubsystem;
    m_fieldOrientedFunction = fieldOrientedFunction;
    m_chassisSpeeds = speeds;
    addRequirements(swerveSubsystem);
  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // Set chassis speed
    ChassisSpeeds finalChassisSpeeds;
    if (m_fieldOrientedFunction) {
      finalChassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(m_chassisSpeeds, m_SwerveSubsystem.getRotation2d());
    } else {
      finalChassisSpeeds = m_chassisSpeeds;
    }

    // Set the swerve module states
    moduleStates = kSwerve.kinematics.toSwerveModuleStates(finalChassisSpeeds);
    logger.logSwerveModuleState("states", moduleStates);
    m_SwerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
