// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.kCoral;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Coral.Coral;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployCoralCmd extends Command {
  private final Coral m_coral;
  private final Elevator m_elevator;
  private double m_scoreLevel;
  private final LEDSubsystem m_ledSubsytem;

  /** Creates a new coralDeployer. */
  public DeployCoralCmd(Coral coral, double scoreLevel, Elevator elevator, LEDSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies
    m_coral = coral;
    m_elevator = elevator;
    m_scoreLevel = scoreLevel;
    m_ledSubsytem = ledSubsystem;
    addRequirements(coral, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_scoreLevel == 1) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L1);
      outake();
    } else if (m_scoreLevel == 2) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L2);
      outake();
      System.out.println("command commanded");
    } else if (m_scoreLevel == 3) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L3);
      outake();
    } else if (m_scoreLevel == 4) {
      m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L4);
      outake();
    }
  }

  private void outake() {
    System.out.println("coralOuttake");
    m_coral.setOuttakeSpeedDuty(.5);
      // Commands.waitUntil(() -> !m_coral.frontCoralSensor.isPressed())
      //   .andThen(() -> m_coral.coralOuttake.getClosedLoopController().setReference(
      //     m_coral.coralOuttake.getEncoder().getPosition()+1, 
      //     ControlType.kPosition));
      // Commands.waitSeconds(.2)
      //   .andThen(() -> m_coral.stopDeployer());
      // this.cancel();
  }
  // Called every time the scheduler runs while the command is scheduled

  boolean sensedCoral = true;

  @Override
  public void execute() {
    m_ledSubsytem.setPatternForDuration(m_ledSubsytem.coralPickup.reversed(), 2);

    if (!m_coral.frontCoralSensor.isPressed() && sensedCoral == true) {
      m_coral.setOutakePos(m_coral.frontMotor.getEncoder().getPosition()+1);
      sensedCoral = false;
    }

    // TODO: this is stupid vvvvvvvvvvvvvvvvvvvvvvv
    if (!m_coral.frontCoralSensor.isPressed() && sensedCoral == false) {
      new Thread(() -> {
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
        m_elevator.setElevatorHeightMotionProfile(kElevator.ElevatorPosition.L1);
        sensedCoral = true;
        this.cancel();
        Thread.currentThread().interrupt();
      }).start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
