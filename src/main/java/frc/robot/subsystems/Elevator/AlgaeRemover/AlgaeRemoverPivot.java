// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator.AlgaeRemover;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAlgaeRemover;
import frc.robot.Constants.kAlgaeRemover.kPivot.RemoverPositions;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class AlgaeRemoverPivot extends SubsystemBase {
  private final SparkMax removerPivotMotor;
  private final SparkMaxConfig config;
  private final SparkClosedLoopController removerPivotPID;

  private final TrapezoidProfile m_profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(50, 50)); // TODO: May need to adjust these values later
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0,0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(0,0);

  private final ArmFeedforward pivotFeedforward =  new ArmFeedforward(0, 0.13, 0.56);

  private final double kDt = 0.02;

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
        .smartCurrentLimit(40) 
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .positionWrappingEnabled(false)
          .outputRange(-1, 1) 
          .pid(.5, 0, 2); 
        // .maxMotion
        //   .maxAcceleration(0)
        //   .maxVelocity(0)
        //   .allowedClosedLoopError(0);
    removerPivotMotor.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    removerPivotPID = removerPivotMotor.getClosedLoopController();

    setPositionTrapazoidal(RemoverPositions.Stowed);
  }

  // set pivot position
  public void setRemoverPos(RemoverPositions angle) {
    double degrees = angle.getDegrees();
    // double rotations = angle * 75.0 / 2.0 / 360;
    removerPivotPID.setReference(degrees, ControlType.kPosition);
  }

  private void setMotorFFandPIDPosition(double removerPosition, double velocitySetpoint) {
    removerPivotPID.setReference(
      removerPosition,
      ControlType.kPosition,
      ClosedLoopSlot.kSlot0);
      // pivotFeedforward.calculate(
      //   removerPosition, 
      //   velocitySetpoint));
  }

  public void setPositionTrapazoidal(RemoverPositions removerPosition) {
    double rotation = removerPosition.getDegrees() / 360;
    m_goal = new TrapezoidProfile.State(rotation, 0);
  }

  // private double calculateVoltage(double goal) {
  //   return voltage
  // }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    setMotorFFandPIDPosition(m_setpoint.position, m_setpoint.velocity);
  }
}
