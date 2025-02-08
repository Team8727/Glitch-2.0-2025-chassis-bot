// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.utilities.SparkConfigurator.getFollowerMax;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import java.util.Set;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kElevator;
import frc.robot.utilities.SparkConfigurator.LogData;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Elevator extends SubsystemBase {

  private final SparkMax elevatorMotorR;
  private final SparkMax elevatorMotorL;
  private final SparkMaxConfig motorRConfig;
  private final SparkClosedLoopController elevatorPID;
  private final DigitalInput limitSwitch;
  private final double elevatorOffset;

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

    motorRConfig = new SparkMaxConfig();
    motorRConfig
      .smartCurrentLimit(40) // TODO: SET ALL OF THIS STUFF
      .idleMode(IdleMode.kBrake)
      .closedLoop
      .velocityFF(0) // Find Using SysId
      .pid(0, 0, 0)
      .maxMotion
      .maxAcceleration(0)
      .maxAcceleration(0)
      .allowedClosedLoopError(0);

    elevatorMotorL = getFollowerMax(
      elevatorMotorR, 
      kElevator.elevatorMotorLCANID, 
      SparkMax.MotorType.kBrushless, 
      true);

    elevatorPID = elevatorMotorR.getClosedLoopController();

    limitSwitch = new DigitalInput(kElevator.limitSwitchDIO);

    elevatorOffset = 0;
  }

  public void stopElevator() {
    elevatorPID.setReference(0, ControlType.kDutyCycle);
  }

  public void setElevatorHeight(kElevator.ElevatorPosition height) {
    // get double from enum
    double targetHeight = height.getRotations();
    run(() -> elevatorPID.setReference(targetHeight, ControlType.kPosition))
    .andThen(() -> {
      if (targetHeight == 0){
        run(() -> elevatorPID.setReference(-30*5, ControlType.kVelocity))
        .until(() -> limitSwitch.get())// TODO: tune probobly
        .andThen(() -> resetElevatorEncoders());
      }
    });
  }

  public void resetElevatorEncoders() {
    elevatorMotorR.getEncoder().setPosition(0);
    elevatorMotorL.getEncoder().setPosition(0);
  }
  // public void setElevatorHeightFF(kElevator.ElevatorPosition height) {
  //   // get double from enum
  //   double targetHeight = height.getRotations();
  //   elevatorPID.setReference(targetHeight, ControlType.kPosition);
  //   // currentHeight = height;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
