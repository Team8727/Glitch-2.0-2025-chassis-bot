// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator.AlgaeRemover;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAlgaeRemover;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class AlgaeRemoverPivot extends SubsystemBase {
  private final SparkMax removerPivotMotor;
  private final SparkMaxConfig config;
  private final SparkClosedLoopController removerPivotPID;

  /** Creates a new AlgaePivot. */
  public AlgaeRemoverPivot() {
    removerPivotMotor =
        getSparkMax(
            kAlgaeRemover.kPivot.removerPivotMotorCANID,
            SparkLowLevel.MotorType.kBrushless,
            false,
            Set.of(),
            Set.of(
                LogData.POSITION,
                LogData.VELOCITY,
                LogData.VOLTAGE,
                LogData.CURRENT));

    config = new SparkMaxConfig();
    config // TODO: config everything
        .smartCurrentLimit(25) 
        .idleMode(IdleMode.kBrake)
        .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .outputRange(-1, 1) 
          .pid(0, 0, 0) 
          .positionWrappingEnabled(false);
        // .maxMotion
        //   .maxAcceleration(0)
        //   .maxVelocity(0)
        //   .allowedClosedLoopError(0);
    removerPivotMotor.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    removerPivotPID = removerPivotMotor.getClosedLoopController();
  }

  // set pivot position
  public void setRemoverPos(double angle) {
    // double rotations = angle * 75.0 / 2.0 / 360;
    removerPivotPID.setReference(angle, ControlType.kPosition);
  }

  // private double calculateVoltage(double goal) {
  //   return voltage
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
