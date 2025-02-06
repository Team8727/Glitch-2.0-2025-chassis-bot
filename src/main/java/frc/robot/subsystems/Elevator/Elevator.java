// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.utilities.SparkConfigurator.getFollowerMax;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import java.util.Set;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kElevator;
import frc.robot.utilities.SparkConfigurator.LogData;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Elevator extends SubsystemBase {
  private final SparkMax elevatorMotorR;
  private final SparkMax elevatorMotorL;
  private final SparkMaxConfig motorRConfig;
  private final SparkMaxConfig motorLConfig;
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorR = getSparkMax(
      kElevator.elevatorMotorRCANID, 
      SparkMax.MotorType.kBrushless,
      true,
      Set.of(),
      Set.of(
        LogData.POSITION,
        LogData.VELOCITY,
        LogData.VOLTAGE,
        LogData.CURRENT));
    elevatorMotorL = getFollowerMax(
      elevatorMotorR, 
      kElevator.elevatorMotorLCANID, 
      SparkMax.MotorType.kBrushless, 
      true);

    motorRConfig = new SparkMaxConfig();
    motorRConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      .closedLoop
      .velocityFF(0)
      .pid(0, 0, 0)
      .maxMotion
      .maxAcceleration(0)
      .maxAcceleration(0)
      .allowedClosedLoopError(0);
    motorLConfig = new SparkMaxConfig();
    motorLConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      .closedLoop
      .velocityFF(0)
      .pid(0, 0, 0)
      .maxMotion
      .maxAcceleration(0)
      .maxAcceleration(0)
      .allowedClosedLoopError(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
