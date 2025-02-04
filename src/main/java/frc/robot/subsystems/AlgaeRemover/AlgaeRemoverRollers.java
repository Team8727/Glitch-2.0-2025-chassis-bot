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
import frc.robot.commands.DriveCmd;
import frc.robot.utilities.SparkConfigurator.LogData;

public class AlgaeRemoverRollers extends SubsystemBase {
  private final SparkMax removerRollerMotor;
  private final SparkMaxConfig config;
  private final SparkClosedLoopController removerRollerPID;
  /** Creates a new AlgaeRemoverRollers. */
  public AlgaeRemoverRollers() {
    removerRollerMotor = getSparkMax(
      kRemover.kRollers.removerRollerMotorCANID, 
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
    .idleMode(IdleMode.kCoast)
    .closedLoop
      .velocityFF(0)// TODO: tune
      .pid(0, 0, 0)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .maxMotion
        .maxAcceleration(0)
        .maxAcceleration(0)
        .allowedClosedLoopError(0);

  removerRollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters); //TODO: Might need to be resetsafe and presistsafe, but nothing is set yet, so I said no
  
  removerRollerPID = removerRollerMotor.getClosedLoopController();
  }

  public void spinnnnnnn(){
    removerRollerPID.setReference(2000, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
