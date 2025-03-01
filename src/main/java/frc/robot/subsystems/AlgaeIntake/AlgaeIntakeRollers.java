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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAlgaeIntake.kAlgaeIntakeRollers;
import frc.robot.utilities.NetworkTableLogger;
import frc.robot.utilities.SparkConfigurator.LogData;
import java.util.Set;

public class AlgaeIntakeRollers extends SubsystemBase {
  private final SparkMax intakeRollerMotor;
  private final SparkMaxConfig config;
  private final DigitalInput algaeCheck;
  public final SparkClosedLoopController rollerPID;
  public boolean isMoving = false;

  /** Creates a new AlgaeIntakeRollers. */
  public AlgaeIntakeRollers() {

// -=-=-=-=-=- rollerMotor Initialization -=-=-=-=-=-|Contructor|

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
                LogData.CURRENT));

// -=-=-=-=-=- pivotMotor config, PID config, and maxMotion constraints config -=-=-=-|Contructor|

    // Setting the output range, PID, and maxMotion constraints for the motor
    config = new SparkMaxConfig(); // TODO: figure out all values (figure out how to do maxvel and
    // maxaccel) (pid is tuned through Rev Hardware Client for onboard PID
    // on motor controller)

    config
      // Motor Config
      .smartCurrentLimit(40) // TODO: figure out what this should be
      .idleMode(IdleMode.kBrake)
      .inverted(true)

      // PID Control
      .closedLoop
        .outputRange(-1, 1) // TODO: currently set to full range
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // MaxMotion Control for more precise position control
        // .maxMotion
        //   .maxVelocity(0)
        //   .maxAcceleration(0)
        //   .allowedClosedLoopError(0);

    // Configuring Motor With Config
    intakeRollerMotor.configure(
        config, 
        ResetMode.kNoResetSafeParameters, 
        PersistMode.kNoPersistParameters);

// -=-=-=-=- Instantiate Intake Sensor -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Contructor|

    algaeCheck = new DigitalInput(kAlgaeIntakeRollers.sensorChannel);

// -=-=-=-=- PID controller for the motor for convenience -=-=-=-|Contructor|

    rollerPID = intakeRollerMotor.getClosedLoopController();
  }

// -=-=-=-=-=-=- Subsystem Methods -=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Subsystem|

  /**
   * Sets the speed of the rollers, implementing built-in PID
   * @param speed : the speed to provide to the motor's built-in PID controller
   * @apiNote This also includes feedforward because we passed in a velocity FF 
   * gain into the built-in controller, and it will automatically use the FF.
   */
  public void setRollerSpeed(double speed) {
    rollerPID.setReference(speed, ControlType.kVelocity); 
  }

  public void setRollerVoltage(double voltage) {
    rollerPID.setReference(voltage, ControlType.kVoltage);
  }

  public void setrollercurrent(double current) {
    rollerPID.setReference(current, ControlType.kCurrent);
  }

  public void stopRollers() {
    setRollerSpeed(0);
  }

  public boolean getAlgaeCheck() {
    return !algaeCheck.get();
  }

// -=-=-=-=-=-=-=-= Logging =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Subsystem|
  
boolean m_shouldLog = false;
  NetworkTableLogger periodicLogger = new NetworkTableLogger(this.getSubsystem().toString());

  /**
   * Whether to log values (like encoder data)
   */
  public void shouldLogValues(boolean shouldLog) {
    m_shouldLog = shouldLog;
  }

  /**
   * Used in subsystem periodic to log and update values
   */
  public void startLogging() { // Only for calling in the periodic of this subsystem
    periodicLogger.logDouble("Motor Current", intakeRollerMotor.getOutputCurrent());
    periodicLogger.logBoolean("Intake Sensor (boolean)", algaeCheck.get());
    // Add other relevant values
  }
// -=-=-=-=-=-=-=-=- Commands -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-|Subsystem|

  public void outtake() {
    run(() -> isMoving = true)
      .andThen(() -> rollerPID.setReference(.3, ControlType.kDutyCycle));
      Commands.waitUntil(() -> !getAlgaeCheck())
        .withTimeout(.5)
      .andThen(() -> rollerPID.setReference(0, ControlType.kDutyCycle))
      .finallyDo(() -> isMoving = false);
  }

  // public Command score() {
  //   return run(() -> setRollerVoltage(-kAlgaeIntakeRollers.scoreVoltage))
  //       .until(() -> !getAlgaeCheck())
  //       .finallyDo(() -> setRollerVoltage(0));
  // }

  private void holdAlgae() {
    if (isMoving == false) {
      if (getAlgaeCheck() == true) {
        rollerPID.setReference(.4, ControlType.kDutyCycle);
        System.out.println("holding");
      } else {
      rollerPID.setReference(0, ControlType.kDutyCycle);
      }
    }
  }

// -=-=-=-=-=-=- Periodic Override -=-=-=-=-=-=-=-=-=-=-=-|Subsystem|
  
  @Override
  public void periodic() {
    holdAlgae();
    // This method will be called once per scheduler run
    if (m_shouldLog) {
      startLogging();
    }
  }
}

