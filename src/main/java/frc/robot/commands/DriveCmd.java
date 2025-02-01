package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCmd extends Command{

    private final SwerveSubsystem m_SwerveSubsystem;
    private final ChassisSpeeds m_speeds;
    private final Supplier<Boolean> m_fieldOrientedFunction;


    public DriveCmd(
            SwerveSubsystem swerveSubsystem,
            ChassisSpeeds speeds,
            Supplier<Boolean> fieldOrientedFunction) {
        m_SwerveSubsystem = swerveSubsystem;
        m_speeds = speeds;
        m_fieldOrientedFunction = fieldOrientedFunction;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        //Set chassis speed 
        ChassisSpeeds finalChassisSpeeds;
        if (m_fieldOrientedFunction.get()) {
            finalChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                m_speeds.vxMetersPerSecond,
                m_speeds.vyMetersPerSecond,
                m_speeds.omegaRadiansPerSecond,
                m_SwerveSubsystem.getRotation2d());
        } else {
            finalChassisSpeeds = new ChassisSpeeds(
                m_speeds.vxMetersPerSecond,
                m_speeds.vyMetersPerSecond,
                m_speeds.omegaRadiansPerSecond);
        }

        // Set the swerve module states
        SwerveModuleState[] moduleStates = kSwerve.kinematics.toSwerveModuleStates(finalChassisSpeeds);

        // output to swerve modules
        m_SwerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.stopModules(); // does nothing
    }

    @Override 
    public boolean isFinished() {
        return false;
    }

}
