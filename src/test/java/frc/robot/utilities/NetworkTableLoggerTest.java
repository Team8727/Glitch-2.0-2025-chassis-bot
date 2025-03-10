package frc.robot.utilities;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.utilities.NetworkTableLogger;

public class NetworkTableLoggerTest {
    private NetworkTableLogger logger;

    private NetworkTableEntry getTableEntry(String key) {
        return logger.getNetworkTable().getEntry(key);
    }

    @BeforeEach
    public void setUp() {
        logger = new NetworkTableLogger("TestLogger");
    }

    @Test
    public void testLogDouble() {
        String key1 = "someValue";
        String key2 = "someOtherValue";

        for (double i = 0.0; i < 10.0; i += 1.0) {
            logger.logDouble(key1, i);
            logger.logDouble(key2, i * 2);
            assertEquals(i, getTableEntry(key1).getDouble(-1));
            assertEquals(i * 2, getTableEntry(key2).getDouble(-1));
        }
    }

    @Test
    public void testLogBoolean() {
        String key1 = "someBoolean";
        String key2 = "someOtherBoolean";

        for (int i = 0; i < 10; ++i) {
            logger.logBoolean(key1, i % 2 == 0);
            logger.logBoolean(key2, i % 2 == 1);
            assertEquals(i % 2 == 0, getTableEntry(key1).getBoolean(false));
            assertEquals(i % 2 == 1, getTableEntry(key2).getBoolean(false));
        }
    }

    @Test
    public void testLogString() {
        String key1 = "someString";
        String key2 = "someOtherString";

        String base = "string";
        for (int i = 0; i < 10; ++i) {
            logger.logString(key1, base + i);
            logger.logString(key2, base + i + 1);
            assertEquals(base + i, getTableEntry(key1).getString(""));
            assertEquals(base + i + 1, getTableEntry(key2).getString(""));
        }
    }

    @Test
    public void testLogInt() {
        String key1 = "someInt";
        String key2 = "someOtherInt";

        for (int i = 0; i < 10; i++) {
            logger.logInt(key1, i);
            logger.logInt(key2, i * 2);
            assertEquals(i, getTableEntry(key1).getInteger(-1));
            assertEquals(i * 2, getTableEntry(key2).getInteger(-1));
        }
    }
}