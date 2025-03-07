package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kElevator.ElevatorPosition;
import frc.robot.commands.RemoveAlgaeCmd;
import frc.robot.commands.SetElevatorHeightCmd;
import frc.robot.commands.AlgaeIntake.IntakeAlgaeCmd;
import frc.robot.commands.AlgaeIntake.ScoreAlgaeCmd;
import frc.robot.commands.Coral.DeployCoralCmd;
import frc.robot.commands.Coral.IntakeCoralCmd;
import frc.robot.commands.Coral.ReindexCoralCmd;
import frc.robot.commands.Coral.RejectCoralCmd;
import frc.robot.commands.DriveCommands.SwerveJoystickCmd;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeRollers;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverPivot;
import frc.robot.subsystems.Elevator.AlgaeRemover.AlgaeRemoverRollers;
import frc.robot.subsystems.Elevator.Coral.Coral;

/**
 * Default teleop controller bindings for the robot.
 */
public class DefaultTeleopControllerBindings implements ControllerBindings {
    private final SwerveSubsystem m_SwerveSubsystem;
    private final AlgaeIntakePivot m_AlgaeIntakePivot;
    private final AlgaeIntakeRollers m_AlgaeIntakeRollers;
    private final Coral m_coral;
    private final Elevator m_elevator;
    private final LEDSubsystem m_ledSubsytem;
    private final boolean m_elevatorSpeedControl;
    private final AlgaeRemoverPivot m_AlgaeRemoverPivot;
    private final AlgaeRemoverRollers m_AlgaeRemoverRollers;

    public DefaultTeleopControllerBindings(
        SwerveSubsystem swerveSubsystem,
        AlgaeIntakePivot AlgaeIntakePivot,
        AlgaeIntakeRollers AlgaeIntakeRollers,
        Coral coral,
        Elevator elevator,
        LEDSubsystem ledSubsystem,
        boolean elevatorSpeedControl,
        AlgaeRemoverPivot algaeRemoverPivot,
        AlgaeRemoverRollers algaeRemoverRollers) {
        m_SwerveSubsystem = swerveSubsystem;
        m_AlgaeIntakePivot = AlgaeIntakePivot;
        m_AlgaeIntakeRollers = AlgaeIntakeRollers;
        m_coral = coral;
        m_elevator = elevator;
        m_ledSubsytem = ledSubsystem;
        m_elevatorSpeedControl = elevatorSpeedControl;
        m_AlgaeRemoverPivot = algaeRemoverPivot;
        m_AlgaeRemoverRollers = algaeRemoverRollers;
    }

    @Override
    public void bind(CommandXboxController controller) {
        m_SwerveSubsystem.setDefaultCommand(
            new SwerveJoystickCmd(
                m_SwerveSubsystem,
                m_elevator,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> controller.getRightX(),
                () -> true,
                () -> m_elevatorSpeedControl));
        /* To Test ChassisSpeeds */
        // controller.a()
        //   .toggleOnTrue(new RunCommand(
        //     () -> 
        //       m_SwerveSubsystem.setModuleStates(
        //         kSwerve.kinematics.toSwerveModuleStates(
        //           new ChassisSpeeds(1,1,1)))));

        // Zero heading
        controller.start().onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroHeading()));

        // // Remove algae L3
        // controller.povUp().onTrue(new removeAlgae(m_AlgaeRemoverPivot, m_AlgeaRemoverRollers, 3, m_elevator));
        // // Remove algae L2
        // controller.povDown().onTrue(new removeAlgae(m_AlgaeRemoverPivot, m_AlgeaRemoverRollers, 2, m_elevator));

        //               coral commands
        // intake coral
        controller.leftTrigger().toggleOnTrue(new IntakeCoralCmd(m_coral, m_elevator, m_ledSubsytem));
        // reindex coral
        controller.povRight().onTrue(new ReindexCoralCmd(m_coral));
        controller.povLeft().onTrue(new RejectCoralCmd(m_coral));
        //deploy coral
        controller.leftBumper().onTrue(new DeployCoralCmd(m_coral, m_ledSubsytem, m_elevator));

        // elevator L1
        controller.x().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L1, m_elevator, m_coral, m_ledSubsytem));
        // elevator L2
        controller.a().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L2, m_elevator, m_coral, m_ledSubsytem));
        // elevator L3
        controller.b().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L3, m_elevator, m_coral, m_ledSubsytem));
        // elevator L4
        controller.y().onTrue(new SetElevatorHeightCmd(ElevatorPosition.L4, m_elevator, m_coral, m_ledSubsytem));

        // zero elevator
        controller.leftStick().and(controller.rightStick()).onTrue(new InstantCommand(() -> m_elevator.resetElevatorEncoders()));

        controller.povUp().whileTrue(new RemoveAlgaeCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers, ElevatorPosition.A3, m_elevator, m_ledSubsytem));
        controller.povDown().whileTrue(new RemoveAlgaeCmd(m_AlgaeRemoverPivot, m_AlgaeRemoverRollers, ElevatorPosition.A2, m_elevator, m_ledSubsytem));


        // Align to pose
        // controller.povLeft().onTrue(m_Autos.align(kPoses.blueFrontLeft).andThen(() -> System.out.println("aligginginsdaod")));

        
        //                algae commands
        // Intake algae
        controller.rightTrigger().whileTrue(new IntakeAlgaeCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers, m_ledSubsytem));
        // deploy algae
        controller.rightBumper().onTrue(new ScoreAlgaeCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers, m_ledSubsytem));
        // controller.rightBumper().onTrue(new ScoreAlgaeProcessorCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers, m_ledSubsytem));

        // // Intake algae
        // controller.rightBumper().onTrue(
        //   new IntakeAlgaeCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers)
        //   .andThen(() -> 
        //     m_ledSubsytem.setPatternForDuration(
        //       m_ledSubsytem.solidRed, 1.5)
        //   ));

        // // Place algae in processor
        // controller.rightTrigger().onTrue(
        //   new ScoreAlgaeProcessorCmd(m_AlgaeIntakePivot, m_AlgaeIntakeRollers)
        //   .andThen(() -> 
        //     m_ledSubsytem.setPatternForDuration(
        //       m_ledSubsytem.solidGreen, 1.5)
        //   ));

        // X configuration
        // controller.x().toggleOnTrue(m_SwerveSubsystem.XPosition());
        // Align to pose
        // controller.b().onTrue(m_Autos.align(kPoses.blueFrontLeft));
        // Xbox Controller Bindings for LED Patterns
        // controller.y().onTrue(new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.rainbow), m_ledSubsytem));
        // controller.b().onTrue(new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.blue), m_ledSubsytem));

        // controller.leftBumper()
        //     .onTrue(
        //         new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.red), m_ledSubsytem));
        // // controller.rightBumper().onTrue(new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.green), m_ledSubsytem));
        // controller.a()
        //     .onTrue(
        //         new InstantCommand(() -> m_ledSubsytem.setPattern(m_ledSubsytem.ace), m_ledSubsytem));
        // controller.y()
        //     .onTrue(
        //         new InstantCommand(
        //             () -> m_ledSubsytem.setPattern(m_ledSubsytem.colorCheck), m_ledSubsytem));

        // .until(() ->
        //     m_PoseEstimatior.get2dPose()
        //       .getTranslation()
        //     .getDistance(
        //       kPoses.blueFrontLeft.getTranslation())
        //     < 0.1
        //   &&
        //     m_PoseEstimatior.get2dPose()
        //       .getRotation()
        //     .minus(
        //       kPoses.blueFrontLeft.getRotation())
        //         .getDegrees()
        //     < 0.1)
        // .andThen(() -> joystickOperated())
    }

    @Override
    public void unbind(CommandXboxController controller) {
        m_SwerveSubsystem.removeDefaultCommand();
    }
    
}
