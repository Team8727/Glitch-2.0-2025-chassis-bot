package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.NetworkTableLogger;

import java.util.function.Supplier;

public class DriveCmd extends SubsystemBase {

  private final SwerveSubsystem m_SwerveSubsystem;
  private final Boolean m_fieldOrientedFunction;
  private SwerveModuleState[] moduleStates = kSwerve.kinematics.toSwerveModuleStates(new ChassisSpeeds());
  private final NetworkTableLogger logger = new NetworkTableLogger("modules");


  public DriveCmd(
      SwerveSubsystem swerveSubsystem,
      Boolean fieldOrientedFunction) {
    m_SwerveSubsystem = swerveSubsystem;
    m_fieldOrientedFunction = fieldOrientedFunction;
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {

    // Set chassis speed
    ChassisSpeeds finalChassisSpeeds;
    if (m_fieldOrientedFunction) {
      finalChassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_SwerveSubsystem.getRotation2d());
    } else {
      finalChassisSpeeds = speeds;
    }

    // Set the swerve module states
    moduleStates = kSwerve.kinematics.toSwerveModuleStates(finalChassisSpeeds);
    logger.logSwerveModuleState("states", moduleStates);
  }

  @Override
  public void periodic() {
    m_SwerveSubsystem.setModuleStates(moduleStates);
  }
}
