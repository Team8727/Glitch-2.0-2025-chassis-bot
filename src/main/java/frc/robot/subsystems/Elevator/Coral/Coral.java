// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator.Coral;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCoral;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class Coral extends SubsystemBase {
  public final SparkMax coralIntake;
  private final SparkMaxConfig intakeConfig;
  public final SparkMax coralOuttake;
  private final SparkMaxConfig outtakeConfig;
  public final SparkLimitSwitch frontCoralSensor;
  public final SparkLimitSwitch backCoralSensor;

  private final NetworkTableLogger logger = new NetworkTableLogger(this.getSubsystem().toString());

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
        .pid(0.5, 0, 0);
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
        .pid(0.5, 0, 0)
;
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
    coralIntake.getClosedLoopController().setReference(speed, ControlType.kDutyCycle);
  }

  public void setOuttakeSpeed(double speed) {
    coralOuttake.getClosedLoopController().setReference(speed, ControlType.kDutyCycle);
  }

  public void stopDeployer() {
    coralIntake.getClosedLoopController().setReference(0, ControlType.kDutyCycle);
    coralOuttake.getClosedLoopController().setReference(0, ControlType.kDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logger.logBoolean(
      "Front Coral Sensor", 
      frontCoralSensor.isPressed());

    logger.logBoolean(
      "Back Coral Sensor", 
      backCoralSensor.isPressed());;
  }
}
