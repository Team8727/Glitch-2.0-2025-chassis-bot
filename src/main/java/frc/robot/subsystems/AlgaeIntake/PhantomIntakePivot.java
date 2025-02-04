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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake.kPivot;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class PhantomIntakePivot extends SubsystemBase {

  private final SparkMax intakePivotMotor;
  private final SparkMaxConfig config;


  private final ArmFeedforward pivotFeedforward;
  private final SparkClosedLoopController pivotPID;

  /** Creates a new PhantomIntake. */
  public PhantomIntakePivot() {

    // =-=-=-=- pivotMotor Initialization -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    intakePivotMotor =
        getSparkMax(
            kPivot.intakePivotMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT)); // TODO: logging everything for now

    // =-=-=-=- pivotMotor PID config and maxMotion Constraints config -=-=-=-=

    // Setting the output range, PID, and maxMotion constraints for the motor
    config = new SparkMaxConfig(); // TODO: figure out all values (figure out how to do maxvel and
    // maxaccel) (pid is tuned through Rev Hardware Client for onboard PID
    // on motor controller)
    config
        .closedLoop
        .outputRange(-1,1) // TODO: this is set to full range of motor speed, might want to scale down to test.
        .pid(0, 0, 0);
    config
        .closedLoop
          .maxMotion
            .maxVelocity(0) // TODO: this is set to zero right now!!
            .maxAcceleration(0);
    intakePivotMotor.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode
            .kNoPersistParameters); // TODO: Might need to be resetsafe and presistsafe, but nothing is set yet, so I said no

    // -=-=-=-=- PID controller for the motor -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    pivotPID = intakePivotMotor.getClosedLoopController();

    // -=-=-=-=- Feedforward (Arm) for the IntakePivot -=-=-=-=-=-=-=-=-=-=-=-=

    pivotFeedforward = new ArmFeedforward(0, 0, 0, 0); // TODO: Calculate these using sysID

    // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  }

  public void setMotorFFPID(
      double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
    pivotPID.setReference(
        pivotFeedforward.calculate(positionRadians, velocityRadPerSec, accelRadPerSecSquared),
        SparkBase.ControlType.kVoltage);

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

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
