// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
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

    frontCoralSensor = new DigitalInput(kCoralIntake.kRollers.frontSensorChannel);

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

    backCoralSensor = new DigitalInput(kCoralIntake.kRollers.backSensorChannel);
  }

  public void setIntakeSpeed(double speed) {
    coralIntake.getClosedLoopController().setReference(speed, ControlType.kVelocity);
  }

  private void setIntakeVoltage(double voltage) {
    coralIntake.getClosedLoopController().setReference(voltage, ControlType.kVoltage);
  }

  public void setOuttakeSpeed(double speed) {
    coralOuttake.getClosedLoopController().setReference(speed, ControlType.kVelocity);
  }

  private void setOuttakeVoltage(double voltage) {
    coralOuttake.getClosedLoopController().setReference(voltage, ControlType.kVoltage);
  }

  public void stopCoralIntake() {
    coralIntake.set(0);
  }

  public void stopCoralOuttake() {
    coralOuttake.set(0);
  }

  public Command stopCoral() {
    return new ParallelCommandGroup(
      new RunCommand(() -> stopCoralIntake()), 
      new RunCommand(() -> stopCoralOuttake()));
  }

  public Command coralIntake() {
    return new RunCommand(
      () -> setIntakeSpeed(kCoralIntake.kRollers.intakeSpeed))
      .until(() -> backCoralSensor.get())
      .andThen(
        new ParallelCommandGroup(
          new RunCommand(() -> setIntakeSpeed(kCoralIntake.kRollers.intakeSpeed)),
          new RunCommand(() -> setIntakeSpeed(kCoralIntake.kRollers.intakeSpeed)))
        .until(() -> !backCoralSensor.get() && frontCoralSensor.get()));
  }

  public Command coralOuttake() {
    return new RunCommand(() -> setOuttakeSpeed(kCoralIntake.kRollers.intakeSpeed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
