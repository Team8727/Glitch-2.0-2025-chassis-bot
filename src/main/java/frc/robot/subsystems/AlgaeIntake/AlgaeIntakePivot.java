// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakePivot;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator.LogData;

import java.util.Set;

public class AlgaeIntakePivot extends SubsystemBase {

  private final SparkMax intakePivotMotor;
  private final SparkMaxConfig config;

  private final ArmFeedforward pivotFeedforward;
  private final SparkClosedLoopController pivotPID;

  private final Encoder pivotEncoder;

  /** Creates a new PhantomIntake. */
  public AlgaeIntakePivot() {

// =-=-=-=- pivotMotor Initialization -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Constructor|

    intakePivotMotor =
        getSparkMax(
            kAlgaeIntakePivot.intakePivotMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT)); // TODO: logging everything for now

// -=-=-=-=- Feedforward (Arm) for the IntakePivot (Not used currently) -=-=-=-=-=-=-|Constructor|

    pivotFeedforward = new ArmFeedforward(0, 0, 0, 0); // TODO: Calculate these using sysID

// =-=-=-=- pivotMotor config, PID config, and maxMotion Constraints config -=-=-=-=-|Constructor|

    // Setting the output range, PID, and maxMotion constraints for the motor
    config = new SparkMaxConfig(); // TODO: figure out all values (figure out how to do maxvel and
    // maxaccel) (pid is tuned through Rev Hardware Client for onboard PID
    // on motor controller)

    config
      // Motor Config
      .smartCurrentLimit(25) // TODO: figure out what this should be
      .idleMode(IdleMode.kBrake)

      // PID Control
      .closedLoop
        .outputRange(-1, 1) // TODO: this is set to full range of motor speed, might want to scale down to test.
        .velocityFF(0)
        .pid(0, 0, 0)

      // MaxMotion Control for more precise position control
        .maxMotion
          .maxVelocity(0) // TODO: this is set to zero right now!!
          .maxAcceleration(0)
          .allowedClosedLoopError(0); // TODO: set this

    // Configuring Motor With Config
    intakePivotMotor.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode
            .kNoPersistParameters); // TODO: Might need to be resetsafe and presistsafe, but nothing
    // is set yet, so I said no

// -=-=-=-=- PID controller for the motor for convenience -=-=-=-=-=-=-=-=|Constructor|

    pivotPID = intakePivotMotor.getClosedLoopController();

// -=-=-=-=- Encoder -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=|Constructor|

    pivotEncoder =
        new Encoder(
            kAlgaeIntakePivot.intakePivotEncoderChannelA,
            kAlgaeIntakePivot
                .intakePivotEncoderChannelB); // TODO: figure out the ports for the encoder

    pivotEncoder.setDistancePerPulse(
        2 * Math.PI
        / (kAlgaeIntakePivot.encoderPulsesPerRevolution * kAlgaeIntakePivot.gearRatio)); // TODO: set these two values if not set
    pivotEncoder.reset();
  }

// -=-=-=--=-=-=-= Logging =-=-=-=-=-=-=-=-=-=-|Subsystem|

  boolean m_shouldLog = false;
  NetworkTableLogger periodicLog = new NetworkTableLogger(this.getSubsystem().toString());

  /**
   * Whether to log values (like encoder data)
   */
  public void shouldLogValues(boolean shouldLog) {
    m_shouldLog = shouldLog;
  }

  /**
   * Used in subsystem periodic to log and update values
   */
  public void startLogging() { // Only for calling in the periodic of this subsystem
    periodicLog.logDouble("Motor Current", intakePivotMotor.getOutputCurrent());
    periodicLog.logDouble("Motor Encoder Value (Relative Encoder):", intakePivotMotor.getEncoder().getPosition());
    periodicLog.logDouble("External Encoder Value:", pivotEncoder.getDistance());
  }

// -=-=-=-=-=-=- Methods -=-=-=-=-=-=-=-=-=-=-|Subsystem|

  public void resetEncoder() {
    pivotEncoder.reset();
  }

  public void setIntakePivotPosition(double positionRadians) {
    pivotPID.setReference(positionRadians, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public void setMotorFFandPID(double positionRadians, double velocityRadPerSec) {
    pivotPID.setReference(
        pivotFeedforward.calculate(positionRadians, velocityRadPerSec),
        SparkBase.ControlType.kVoltage);
  }

    // pivotMotor.setReference() //To set built-in PID (maybe put the feedforward calculation in
    // here as parameter?)
    // pivotFeedforward.calculate(positionRadians, velocityRadPerSec);  //For velocity and position
    // control, acceleration assumed to be 0
    // pivotFeedforward.calculate(positionRadians, velocityRadPerSec, accelRadPerSecSquared);  //For
    // control of all three values
    // pivotFeedforward.calculate(currentAngle, currentVelocity, nextVelocity, dt)  // For velocity
    // control
    // pivotFeedforward.calculateWithVelocities(currentAngle, currentVelocity, nextVelocity); //
    // Other method of velocity control

// -=-=-=-=-=-=- Commands -=-=-=-=-=-=-=-=-=-=-|Subsystem|



// -=-=-=-=-=-=- Less Used Methods -=-=-=-=-=-=-|Subsystem|

  // For SysId
  public void setIntakeRawVoltage(double voltage) {
    intakePivotMotor.setVoltage(voltage);
  }

// -=-=-=-=-=-=- Periodic Override -=-=-=-=-=-=-=-=-|Subsystem|

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_shouldLog) {
      startLogging();
    }
  }
}
