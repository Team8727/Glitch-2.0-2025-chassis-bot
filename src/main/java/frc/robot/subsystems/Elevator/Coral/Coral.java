// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator.Coral;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCoral;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class Coral extends SubsystemBase {
  private final SparkMax coralIntake;
  private final SparkMaxConfig intakeConfig;
  private final SparkMax coralOuttake;
  private final SparkMaxConfig outtakeConfig;
  public final SparkLimitSwitch frontCoralSensor;
  public final SparkLimitSwitch backCoralSensor;

  /** Creates a new Coral. */
  public Coral() {
    coralIntake =
        getSparkMax(
            kCoral.intakeRollerMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT));
    intakeConfig = new SparkMaxConfig();
    intakeConfig // TODO: tune configs
        .smartCurrentLimit(25)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .velocityFF(0)
        .pid(0, 0, 0);
        // .maxMotion
        // .maxAcceleration(0)          // Disabling max motion for these rollers (no need to be very precise). 
        // .maxAcceleration(0)
        // .allowedClosedLoopError(0);

    coralIntake.configure(
        intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    coralOuttake =
        getSparkMax(
            kCoral.outtakeRollerMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT));

    outtakeConfig = new SparkMaxConfig();
    outtakeConfig// TODO: tune configs
        .smartCurrentLimit(25) 
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .velocityFF(0) 
        .pid(0, 0, 0);
        // .maxMotion
        // .maxAcceleration(0)          // Disabling max motion for these rollers (no need to be very precise). 
        // .maxAcceleration(0)
        // .allowedClosedLoopError(0);

    coralOuttake.configure(
        outtakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    frontCoralSensor = coralOuttake.getForwardLimitSwitch();
    backCoralSensor = coralOuttake.getReverseLimitSwitch();
  }

  public void setIntakeSpeed(double speed) {
    coralIntake.getClosedLoopController().setReference(speed, ControlType.kVelocity);
  }

  public void setOuttakeSpeed(double speed) {
    coralOuttake.getClosedLoopController().setReference(speed, ControlType.kVelocity);
  }

  public void stopDeployer() {
    setIntakeSpeed(0);
    setOuttakeSpeed(0);
  }
  public Command coralOuttake(double speed) {
    return new RunCommand(() -> setOuttakeSpeed(kCoral.outtakeSpeed))
        .until(() -> !frontCoralSensor.isPressed())
        .andThen(() -> stopDeployer());
  }

  public Command coralIntake(double speed) {
    return new RunCommand(() -> setIntakeSpeed(kCoral.intakeSpeed))
        .until(() -> backCoralSensor.isPressed())
          .andThen(() -> setIntakeSpeed(speed))
          .andThen(() -> setOuttakeSpeed(speed))
        .until(() -> !backCoralSensor.isPressed())
          .andThen(() -> stopDeployer());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
