// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import static frc.robot.utilities.SparkConfigurator.getFollowerMax;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import java.util.Set;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kElevator;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator.LogData;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Elevator extends SubsystemBase {

  private final SparkMax elevatorMotorR;
  private final SparkMax elevatorMotorL;
  private final SparkMaxConfig motorRConfig;
  private final SparkClosedLoopController elevatorPID;
  private final DigitalInput limitSwitch;
  private kElevator.ElevatorPosition targetHeight;
  private double targetRotations;
  private NetworkTableLogger logger = new NetworkTableLogger("Elevator");

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
    motorRConfig // TODO: SET ALL OF THIS STUFF
      .smartCurrentLimit(60) 
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .closedLoop
      // .velocityFF(0) // Find Using SysId
      .pid(2, 0, 0)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      // .maxMotion
      // .maxVelocity(0)
      // .maxAcceleration(0)
      // .allowedClosedLoopError(0);
    elevatorMotorR.configure(motorRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorMotorL = getFollowerMax(
      elevatorMotorR, 
      kElevator.elevatorMotorLCANID, 
      SparkMax.MotorType.kBrushless, 
      false);

    elevatorPID = elevatorMotorR.getClosedLoopController();

    limitSwitch = new DigitalInput(kElevator.limitSwitchDIO);
  }

  public void stopElevator() {
    elevatorPID.setReference(0, ControlType.kDutyCycle);
  }

  public void setElevatorHeight(kElevator.ElevatorPosition height) {
    // get double from enum
    targetHeight = height;
    targetRotations = height.getOutputRotations();
    System.out.println("numbers" + targetRotations);
    elevatorMotorR.getClosedLoopController().setReference(targetRotations, ControlType.kPosition);

    // run(() -> elevatorPID.setReference(targetRotations, ControlType.kPosition))
    //   .until(limitSwitch::get)
    //   .andThen(() -> resetElevatorEncoders())
    //   .withTimeout(1);// TODO: limit tune probobly
    
    // System.out.println("move");
    
    // if (targetHeight == kElevator.ElevatorPosition.HOME && !limitSwitch.get()) {
    //   run(() -> elevatorPID.setReference(-30*5, ControlType.kVelocity)) //TODO: this ends instantly so until does nothing
    //   .until(() -> limitSwitch.get())
    //   .andThen(() -> {
    //     elevatorPID.setReference(0, ControlType.kVelocity);
    //     resetElevatorEncoders();
    //   });
    // }
    // TODO: current zeroing?
    // if (targetHeight == kElevator.ElevatorPosition.HOME) {
    //   run(() -> elevatorPID.setReference(-30 * 5, ControlType.kVelocity))
    //   .until(() -> elevatorMotorL.getOutputCurrent() >= 60)
    //   .andThen(() -> {
    //     elevatorPID.setReference(0, ControlType.kVelocity);
    //     resetElevatorEncoders();
    //   });
    // }
  };

  public kElevator.ElevatorPosition getElevatorSetPosition() {
    return targetHeight;
  }

  public double getElevatorHeight() {
    try {
      return elevatorMotorR.getEncoder().getPosition();
    } catch (NullPointerException e) {
      return 0;
    }
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
    logger.logDouble("Lcurrent", elevatorMotorL.getOutputCurrent());
    logger.logDouble("Lrotations", elevatorMotorL.getEncoder().getPosition());
    logger.logDouble("Rcurrent", elevatorMotorR.getOutputCurrent());
    logger.logDouble("Rrotations", elevatorMotorR.getEncoder().getPosition());

    // This method will be called once per scheduler run
  }
}
