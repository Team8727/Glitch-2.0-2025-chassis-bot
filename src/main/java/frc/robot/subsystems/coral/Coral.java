// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.Set;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.kCoralIntake;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkConfigurator.LogData;

public class Coral extends SubsystemBase {
  public SparkMax coralIntake;
  public SparkMaxConfig intakeConfig;
  public SparkMax coralOuttake;
  public SparkMaxConfig outtakeConfig;
  public DigitalInput frontCoralSensor;
  public DigitalInput backCoralSensor;
  /** Creates a new Coral. */
  public Coral() {
    coralIntake = getSparkMax(
      kCoralIntake.kRollers.intakeRollerMotorCANID, 
      SparkMax.MotorType.kBrushless, 
      false,
      Set.of(),
      Set.of(LogData.CURRENT, LogData.VOLTAGE));

    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);

    coralIntake.configure(intakeConfig, 
    ResetMode.kNoResetSafeParameters, 
    PersistMode.kNoPersistParameters);

    coralOuttake = getSparkMax(
      kCoralIntake.kRollers.outtakeRollerMotorCANID, 
      SparkMax.MotorType.kBrushless, 
      false,
      Set.of(),
      Set.of(LogData.CURRENT, LogData.VOLTAGE));
    
    outtakeConfig = new SparkMaxConfig();
    outtakeConfig.idleMode(IdleMode.kBrake);

    coralOuttake.configure(outtakeConfig,
    ResetMode.kNoResetSafeParameters,
    PersistMode.kNoPersistParameters);
  }

  private void setIntakeSpeed(double speed) {
    coralIntake.set(speed);
  }

  private void setIntakeVoltage(double voltage) {
    coralIntake.setVoltage(voltage);
  }

  private void setOuttakeSpeed(double speed) {
    coralOuttake.set(speed);
  }

  private void setOuttakeVoltage(double voltage) {
    coralOuttake.setVoltage(voltage);
  }

  public void stopCoralIntake() {
    coralIntake.set(0);
  }

  public void stopCoralOuttake() {
    coralOuttake.set(0);
  }

  public Command coralIntake() {
    return new RunCommand(
      () -> setIntakeVoltage(-kCoralIntake.kRollers.intakeVoltage), this)
      .until(() -> backCoralSensor.get())
      .andThen(
        new ParallelCommandGroup(
          new RunCommand(() -> setIntakeVoltage(-kCoralIntake.kRollers.intakeVoltage), this),
          new RunCommand(() -> setOuttakeVoltage(-kCoralIntake.kRollers.intakeVoltage), this))
        .until(() -> !backCoralSensor.get() && frontCoralSensor.get()));
  }

  public Command coralOuttake() {
    return new RunCommand(() -> setOuttakeVoltage(-kCoralIntake.kRollers.outtakeVoltage), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
