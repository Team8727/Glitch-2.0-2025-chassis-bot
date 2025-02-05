// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeRemover;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;
import java.util.Set;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kRemover;
import frc.robot.utilities.SparkConfigurator.LogData;

public class AlgaeRemoverPivot extends SubsystemBase {
  private final SparkMax removerPivotMotor;
  private final SparkMaxConfig config;
  private final SparkClosedLoopController removerPivotPID;


  /** Creates a new AlgaePivot. */
  public AlgaeRemoverPivot() {
    removerPivotMotor = getSparkMax(
        kRemover.kPivot.removerPivotMotorCANID, 
        SparkLowLevel.MotorType.kBrushless, 
        false, 
        Set.of(), 
        Set.of(
          LogData.POSITION, 
          LogData.VELOCITY, 
          LogData.VOLTAGE, 
          LogData.CURRENT)); //TODO: logging everything for now

    config = new SparkMaxConfig();
    config
      .smartCurrentLimit(25)// TODO: figure out what this should be
      .idleMode(IdleMode.kBrake)
      .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(-1, 1)// TODO: figure out what these should be
        .pid(0, 0, 0)// TODO: figure out what these should be
        .maxMotion// TODO: figure out what these should be
          .maxAcceleration(0)
          .maxVelocity(0)
          .allowedClosedLoopError(0);
    removerPivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); //TODO: Might need to be resetsafe and presistsafe, but nothing is set yet, so I said no

    removerPivotPID = removerPivotMotor.getClosedLoopController();
  //75:2 reduction 
  //63:720 reduction
  }
  // set pivot position
  public void setRemoverPos(double goal) {
    removerPivotPID.setReference(goal, ControlType.kPosition);
  }
  // private double calculateVoltage(double goal) {
  //   return voltage
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
