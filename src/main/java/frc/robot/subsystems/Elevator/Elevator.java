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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import static edu.wpi.first.units.Units.Volt;
import static frc.robot.utilities.SparkConfigurator.getFollowerMax;
import static frc.robot.utilities.SparkConfigurator.getSparkMax;

import java.util.Set;
import java.util.logging.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
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

//-=-=-=-=-=-=-=-=-TrapezoidProfile=-=-=-=-=-=-=-=- /

  private static double kDt = 0.02;

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile =               //in/s
      new TrapezoidProfile(new TrapezoidProfile.Constraints(103.33, 307.85)); //TODO: SET THESE
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=- 


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
      .smartCurrentLimit(65, 65)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .closedLoop
      .velocityFF(0) // Find Using SysId
      .pid(.25, 0, 0)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      // .maxMotion
      // .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
      // .maxVelocity(4771.41593898)
      // .maxAcceleration(236.923835739)
      // .allowedClosedLoopError(0);//DO NOT CHANGE THEIS UNLESS YOURE 100000% SURE YOU KNOW WHAT YOUR DOING PLEAS LISTE TO THIS WARNEING OR ELCE YOU WILL DIE IM NOT EVEN JOKING PLEAS DONT CHANG THIS.
    elevatorMotorR.configure(motorRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorMotorL = getFollowerMax(
      elevatorMotorR, 
      kElevator.elevatorMotorLCANID, 
      SparkMax.MotorType.kBrushless, 
      true);

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

  public void setElevatorHeightMotionProfile(kElevator.ElevatorPosition height_chosen) {
    // get double from enum
    targetHeight = height_chosen;
    targetRotations = height_chosen.getOutputRotations();
        
    // Position zero with zero velocity.
    m_goal = new TrapezoidProfile.State(targetRotations, 0);

          //elevatorMotorR.getClosedLoopController().setReference(targetRotations, ControlType.kPosition);

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

          // Retrieve the profiled setpoint for the next timestep. This setpoint moves
          // toward the goal while obeying the constraints.

  }

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

  // Creates a SysIdRoutine
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volt)), null, this)
    );

  private void runVolts(double voltage) {
    elevatorMotorR.setVoltage(voltage);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    logger.logDouble("Rrotations", elevatorMotorR.getEncoder().getPosition());
    logger.logDouble("Rcurrent", elevatorMotorR.getOutputCurrent());
    
    SmartDashboard.putNumber("setpos_smart", targetRotations);
    logger.logDouble("setpos", 7.23663);


    //-=-=-=-=-=-=-=- Trapezoid Profile -=-=-=-=-=-=-=-

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    // Send setpoint to offboard controller PID (I made this in periodic so when the setpositionTrapezoidProfile Method is updated it runs the elevator)
    elevatorMotorR.getClosedLoopController().setReference(m_setpoint.position, ControlType.kPosition);
  }
}
