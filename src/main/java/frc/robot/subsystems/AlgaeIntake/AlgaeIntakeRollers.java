// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakeRollers;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class AlgaeIntakeRollers extends SubsystemBase {
  public SparkMax intakeRollerMotor;
  public SparkMaxConfig intakeRollerConfig;
  public DigitalInput algaeCheck;

  /** Creates a new AlgaeIntakeRollers. */
  public AlgaeIntakeRollers() {
    intakeRollerMotor =
        getSparkMax(
            kAlgaeIntakeRollers.rollerMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(LogData.CURRENT, LogData.VOLTAGE));

    intakeRollerConfig = new SparkMaxConfig();
    intakeRollerConfig.idleMode(IdleMode.kBrake);

    intakeRollerMotor.configure(
        intakeRollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    algaeCheck = new DigitalInput(kAlgaeIntakeRollers.sensorChannel);
  }

  public void setRollerSpeed(double speed) {
    intakeRollerMotor.set(
        speed); // probably should use the intakePivotReduction here but I don't know how to do that
    // properly
  }

  public void setRollerVoltage(double voltage) {
    intakeRollerMotor.setVoltage(voltage);
  }

  public void stopRollers() {
    intakeRollerMotor.set(0);
  }

  public boolean getAlgaeCheck() {
    return algaeCheck.get();
  }

  public Command intake() {
    return run(() -> setRollerVoltage(-kAlgaeIntakeRollers.intakeVoltage))
        .until(() -> getAlgaeCheck())
        .andThen(
            run(() -> setRollerVoltage(-kAlgaeIntakeRollers.intakeVoltage))
                .withTimeout(0.5)) // TODO: this additional time may have to be modified or removed
        .finallyDo(() -> setRollerVoltage(0));
  }

  public Command outtake() {
    return run(() -> setRollerVoltage(-kAlgaeIntakeRollers.outtakeVoltage))
        .until(() -> !getAlgaeCheck())
        .finallyDo(() -> setRollerVoltage(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
