// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  public SparkMax coralIntake;
  public SparkMaxConfig intakeConfig;
  public SparkMax coralOuttake;
  public SparkMaxConfig outtakeConfig;
  public DigitalInput frontCoralSensor;
  public DigitalInput backCoralSensor;
  /** Creates a new Coral. */
  public Coral() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
