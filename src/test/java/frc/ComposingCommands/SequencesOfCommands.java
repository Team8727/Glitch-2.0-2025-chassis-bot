package frc.robot.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SequencesOfCommands {
    private class SetValueSubsystem extends SubsystemBase {
        private int m_value = 0;

        public SetValueSubsystem() {
        }

        public void setValue(int value) {
            m_value = value;
        }

        public int getValue() {
            return m_value;
        }
    }

    @Test
    public void testSequentialInstantCommands() {
        SetValueSubsystem setValueSubsystem = new SetValueSubsystem();

        final double kWaitTime = 0.5;
        SequentialCommandGroup group = new SequentialCommandGroup(
            new InstantCommand(() -> setValueSubsystem.setValue(1)).ignoringDisable(true),
            new WaitCommand(kWaitTime),
            new InstantCommand(() -> setValueSubsystem.setValue(2)).ignoringDisable(true)
        );
        group.schedule();

        CommandScheduler.getInstance().run();

        // should be 1 after first execute
        assertEquals(1, setValueSubsystem.getValue());

        Timer timer = new Timer();
        timer.start();

        while (timer.get() < kWaitTime) {
            try {
                Thread.sleep(20); // Sleep for 20 milliseconds to simulate periodic updates
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            CommandScheduler.getInstance().run();
        }

        // should be 2 after waiting
        assertEquals(2, setValueSubsystem.getValue());
    }
}