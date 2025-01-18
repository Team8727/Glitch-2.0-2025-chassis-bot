package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.MAXSwerve;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kSwerve.kModule;
import frc.robot.utilities.NetworkTableLogger;

public class SwerveSubsystem extends SubsystemBase{
    private final MAXSwerve frontLeftModule = new MAXSwerve(
        kSwerve.CANID.frontLeftDrive,
        kSwerve.CANID.frontLeftSteer,
        kSwerve.Offsets.frontLeft);
    private final MAXSwerve backLeftModule = new MAXSwerve(
        kSwerve.CANID.backLeftDrive,
        kSwerve.CANID.backLeftSteer,
        kSwerve.Offsets.backLeft);
    private final MAXSwerve backRightModule = new MAXSwerve(
        kSwerve.CANID.backRightDrive,
        kSwerve.CANID.backRightSteer,
        kSwerve.Offsets.backRight);
    private final MAXSwerve frontRightModule = new MAXSwerve(
        kSwerve.CANID.frontRightDrive,
        kSwerve.CANID.frontRightSteer,
        kSwerve.Offsets.frontRight);
    
    public final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

    NetworkTableLogger networkTableLogger = new NetworkTableLogger(this.getName().toString());

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
        frontLeftModule.getPositon(),
        backLeftModule.getPositon(),
        backRightModule.getPositon(),
        frontRightModule.getPositon()
    };

    Pose3d pose3d = new Pose3d();
    SwerveDrivePoseEstimator3d swervePoseEstimator = new SwerveDrivePoseEstimator3d(
        kSwerve.kinematics,
        navX.getRotation3d(),
        modulePositions, 
        pose3d);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        navX.reset();
    }

    //maybe = get corrected steer
    public double getHeading() {
        return Math.IEEEremainder(navX.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    @Override 
    public void periodic() {
        networkTableLogger.logDouble("robotHeading", getHeading());
    }
    
    public Command XPosition() {
        return run(() -> {
            frontLeftModule.setX();
            frontRightModule.setX();
            backLeftModule.setX();
            backRightModule.setX();
        });
    }

    public void stopModules() {

    }

    public void setModuleStates(SwerveModuleState[] desiredState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, kModule.maxWheelSpeed);
        frontLeftModule.setTargetState(desiredState[0], true, true);
        frontRightModule.setTargetState(desiredState[1], true, true);
        backLeftModule.setTargetState(desiredState[2], true, true);
        backRightModule.setTargetState(desiredState[3], true, true);
    }   
}
