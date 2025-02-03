// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeRemover;

import java.util.Set;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kRemover;
import frc.robot.utilities.SparkConfigurator.LogData;
import frc.robot.utilities.SparkConfigurator.Sensors;

public class AlgaePivot extends SubsystemBase {
  private final SparkMax removerPivotMotor;
  private final SparkMaxConfig config;

  /** Creates a new AlgaePivot. */
  public AlgaePivot() {
    removerPivotMotor = getSparkMax(
        kRemover.kPivot.removerPivotMotorCANID, 
        SparkLowLevel.MotorType.kBrushless, 
        false, 
        Set.of(), 
        Set.of(LogData.POSITION, LogData.VELOCITY, LogData.VOLTAGE, LogData.CURRENT)); //TODO: logging everything for now

    config = new SparkMaxConfig();
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .outputRange(0, 0)
      .pid(0, 0, 0);
    config.idleMode(IdleMode.kBrake);
    // TODO: current limits
    removerPivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); //TODO: Might need to be resetsafe and presistsafe, but nothing is set yet, so I said no

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
