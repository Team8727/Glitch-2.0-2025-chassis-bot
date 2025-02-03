// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import frc.robot.Constants.kIntake.kPivot;
import frc.robot.utilities.SparkConfigurator.LogData;

import java.util.Set;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhantomIntakePivot extends SubsystemBase {

  private final SparkMax intakePivotMotor;
  private final SparkMaxConfig config;

  private final ArmFeedforward pivotFeedforward;

  /** Creates a new PhantomIntake. */
  public PhantomIntakePivot() {

  //=-=-=-=- intakePivotMotor Initialization -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    intakePivotMotor = 
      getSparkMax(
          kPivot.intakePivotMotorCANID, SparkLowLevel.MotorType.kBrushless, false, 
          Set.of(), 
          Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE, LogData.CURRENT)); //TODO: logging everything for now

  //=-=-=-=- intakePivotMotor PID config and maxMotion Constraints config -=-=-=-=

    config = new SparkMaxConfig(); //TODO: figure out all values (figure out how to do maxvel and maxaccel) (pid is tuned through Rev Hardware Client for onboard PID on motor controller)
    config.closedLoop
      .outputRange(0, 0)
      .pid(0, 0, 0);
    config.closedLoop.maxMotion
      .maxVelocity(0)
      .maxAcceleration(0);
    intakePivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); //TODO: Might need to be resetsafe and presistsafe, but nothing is set yet, so I said no

  //-=-=-=-=- Feedforward (Arm) for the IntakePivot -=-=-=-=-=-=-=-=-=-=-=-=

    pivotFeedforward = new ArmFeedforward(0, 0, 0, 0); //TODO: Calculate these using sysID
 
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  }

  public void calculateVoltage(double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
    // pivotFeedforward.calculate(positionRadians, velocityRadPerSec);  //For velocity and position control, acceleration assumed to be 0
    // pivotFeedforward.calculate(positionRadians, velocityRadPerSec, accelRadPerSecSquared);  //For control of all three values
    // pivotFeedforward.calculate(currentAngle, currentVelocity, nextVelocity, dt)  // For velocity control 
    // pivotFeedforward.calculateWithVelocities(currentAngle, currentVelocity, nextVelocity); // Other method of velocity control
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
