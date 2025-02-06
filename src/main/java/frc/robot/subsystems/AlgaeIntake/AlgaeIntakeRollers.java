// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakeRollers;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class AlgaeIntakeRollers extends SubsystemBase {
  public final SparkMax intakeRollerMotor;
  public final SparkMaxConfig config;
  public final DigitalInput algaeCheck;
  public final SparkClosedLoopController rollerPID;

  /** Creates a new AlgaeIntakeRollers. */
  public AlgaeIntakeRollers() {
    intakeRollerMotor =
        getSparkMax(
            kAlgaeIntakeRollers.rollerMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT)); // TODO: logging everything for now

    config = new SparkMaxConfig();
    config
      .smartCurrentLimit(25) // TODO: figure out what this should be
      .idleMode(IdleMode.kBrake)
      .closedLoop
      .velocityFF(0) // TODO: tune
      .pid(0, 0, 0)
      .maxMotion
      .maxAcceleration(0)
      .maxAcceleration(0)
      .allowedClosedLoopError(0);

    intakeRollerMotor.configure(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kNoPersistParameters);

    algaeCheck = new DigitalInput(kAlgaeIntakeRollers.sensorChannel);

    rollerPID = intakeRollerMotor.getClosedLoopController();
  }

  public void setRollerSpeed(double speed) {
    rollerPID.setReference(speed, ControlType.kVelocity); 
  }

  public void setRollerVoltage(double voltage) {
    rollerPID.setReference(voltage, ControlType.kVoltage);
  }

  public void stopRollers() {
    setRollerSpeed(0);
  }

  public boolean getAlgaeCheck() {
    return algaeCheck.get();
  }

  public Command outtake() {
    return run(() -> setRollerSpeed(-kAlgaeIntakeRollers.outtakeSpeed))
        .until(() -> !getAlgaeCheck())
          .withTimeout(.5)
        .finallyDo(() -> setRollerSpeed(0));
  }

  public Command intake() {
    return run(() -> setRollerSpeed(kAlgaeIntakeRollers.intakeSpeed))
        .until(() -> getAlgaeCheck())
        .andThen(
            run(() -> setRollerSpeed(kAlgaeIntakeRollers.intakeSpeed))
                .withTimeout(0.5)) // TODO: this additional time may have to be modified or removed
        .finallyDo(() -> setRollerSpeed(0));
  }

  public Command score() {
    return run(() -> setRollerVoltage(-kAlgaeIntakeRollers.scoreVoltage))
        .until(() -> !getAlgaeCheck())
        .finallyDo(() -> setRollerVoltage(0));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
