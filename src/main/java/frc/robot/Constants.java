// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.Matrix;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static int configurationSetRetries = 5;

  public static class kOI {
    public static double translationDeadzone = 0.08;
    public static double rotationDeadzone = 0.08;
  }
  
  // Swerve subsystem constants (module constants included)
  public static class kSwerve {
    // Chassis dimensions from wheel center to center (meters)
    public static double width = Units.inchesToMeters(23);
    public static double length = width;

    // Speed & accel limits (m/s, rad/s, m/s^2 & rad/s^2)
    // public static double maxTransSpeed = 1;
    // public static double maxAngSpeed = .3 * Math.PI;

    // public static double maxTransAccel = 1.35 * 9.81;
    // public static double maxAngAccel = 10 * 2 * Math.PI;
    public static double maxTransSpeed = .1;
    public static double maxAngSpeed = .1 * Math.PI;

    public static double maxTransAccel = .1 * 9.81;
    public static double maxAngAccel = .1 * 2 * Math.PI;

    // Operator interface constants
    public static class Teleop {
      public static double translationGain = 1;
      public static double rotationGain = 0.7;

      public static boolean closedLoop = false;
    }

    // NavX
    public static boolean invertGyro = false;
    public static Port navxPort = Port.kMXP;

    // Swerve uses ccw+ angular quanities and a coordinate plane with 0,0 at the robot's center
    // , forward is +x, and a module order based on the quadrant system (front left is first)
    public static SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(length / 2, width / 2),
            new Translation2d(length / 2, -width / 2),
            new Translation2d(-length / 2, width / 2),
            new Translation2d(-length / 2, -width / 2));

    // Module angular offsets (rad)
    public static class Offsets {
      public static double frontLeft = 0;
      public static double backLeft = Math.PI / 2;
      public static double backRight = Math.PI;
      public static double frontRight = -Math.PI / 2;
    }

    // Controller PID values for x/y translation, and z rotation
    public static class Auton {
      public static final double angP = 4;
      public static final double angD = 0;

      public static final double maxAccel = 0.1;
      public static final double maxVel = 0.1;
      public static final double maxAngAccel = 0.1 * kSwerve.maxAngAccel;
      public static final double maxAngVel = 0.1 * kSwerve.maxAngSpeed;
      

      public static final double transP = 12;

      public static final double maxOnTheFlyVel = 3;
      public static final double maxOnTheFlyAcc = 3;

      public static final HolonomicDriveController controller =
          new HolonomicDriveController(
              new PIDController(Auton.transP, 0.0, 0.0),
              new PIDController(angP, 0.0, angD),
              new ProfiledPIDController(0, 0, 0, // TODO: tune this also figure out what it is
                new Constraints(maxVel, maxAccel)));

      // public static final HolonomicPathFollowerConfig pathFollowConfig =
      //     new HolonomicPathFollowerConfig(
      //         new PIDConstants(Auton.transP, 0.0, 0), // Translation PID constants
      //         new PIDConstants(angP, 0.0, angD), // Rotation PID constants
      //         kModule.maxWheelSpeed, // Max module speed, in m/s
      //         Math.pow(
      //             Math.pow(kSwerve.width / 2, 2) + Math.pow(kSwerve.length / 2, 2),
      //             1 / 2), // Drive base radius in meters. Distance from robot center to furthest
      //         // module.
      //         new ReplanningConfig(false, false));
    }

    public static class kModule {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
      public static final int drivePinionTeeth = 14;
      public static final boolean invertSteerEncoder = true;

      // Controls Constants
      public static class kDrive {
        public static final double kP = 0.25;
        public static final double kD = 0.05;
        public static final double kS = 0.068841;
        public static final double kV = 2.4568;
        public static final double kA = 0.22524;
        public static final double minOutput = -1;
        public static final double maxOutput = 1;
      }

      public static class kSteer {
        public static final double kP = 2.5;
        public static final double kD = 0;
        public static final double minOutput = -1;
        public static final double maxOutput = 1;
      }

      // Motor configs
      public static final IdleMode drivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode steeringMotorIdleMode = IdleMode.kBrake;

      public static final int driveSmartCurrentLimit = 50; // amps
      public static final int driveMaxCurrent = 80; // amps
      public static final int steerSmartCurrentLimit = 20; // amps
      public static final int steerMaxCurrent = 35; // amps

      // Physical dimensions/values
      public static final double wheelDiameter = 0.97 * Units.inchesToMeters(3);
      public static final double wheelCircumference = wheelDiameter * Math.PI; // meters
      public static final double driveMotorReduction = (45.0 * 22) / (drivePinionTeeth * 15);
      public static final double steerMotorReduction = 9424.0 / 203.0;

      // Motor physics
      public static final double neoFreeSpeed = 5820.0 / 60; // rot/s
      public static final double neoFreeCurrent = 1.7; // amps
      public static final double neoStallTorque = 3.28; // Nm
      public static final double neoStallCurrent = 181; // amps

      public static final double neo550FreeSpeed = 11710.0 / 60;
      public static final double neo550FreeCurrent = 1.1;
      public static final double neo550StallTorque = 1.08;
      public static final double neo550StallCurrent = 111;

      public static final double maxWheelSpeed =
          (neoFreeSpeed / driveMotorReduction) * (wheelDiameter * Math.PI); // m/s

      // Encoders
      public static final double drivingEncoderPositionFactor =
          (wheelDiameter * Math.PI) / driveMotorReduction; // meters
      public static final double drivingEncoderVelocityFactor =
          ((wheelDiameter * Math.PI) / driveMotorReduction) / 60.0; // m/s

      public static final double steeringEncoderPositionFactor = (2 * Math.PI); // radians
      public static final double steeringEncoderVelocityFactor = (2 * Math.PI) / 60.0; // rad/s

      public static final double steeringEncoderPositionPIDMinInput = 0; // radians
      public static final double steeringEncoderPositionPIDMaxInput =
          steeringEncoderPositionFactor; // radians
    }

    // Motor CAN IDs
    public static class CANID {
      public static int frontLeftDrive = 3;//
      public static int frontLeftSteer = 2;//
      public static int backLeftDrive = 9;//
      public static int backLeftSteer = 8;//
      public static int backRightDrive = 7;//
      public static int backRightSteer = 6;//
      public static int frontRightDrive = 5;
      public static int frontRightSteer = 4;//
    }
  }
  public static class kVision {
    // Vision
    public static final Transform3d camera1Position =
        new Transform3d(
            new Translation3d(0,0, 0),
            new Rotation3d(0, 0, 0));
    public static final Transform3d camera2Position =
        new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));
    public static final Transform3d camera3Position =
        new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));
    public static final Transform3d camera4Position =
        new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));
    
    public static final Matrix<N3, N1> stateStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.02, 0.02, 0.01);
    public static final Matrix<N3, N1> visionStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.03, 0.03, 0.25);
    public static final double visionScalingFactor = 2.3; // scaling factor applied to the visionStdDevs per meter bigger means trust less at a
    // distance
  }
}
