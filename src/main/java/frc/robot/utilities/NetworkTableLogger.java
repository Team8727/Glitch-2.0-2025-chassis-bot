package frc.robot.utilities;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.kElevator.ElevatorPosition;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Custom NetworkTable logger created by Glitch 2.0 in 2025. This logger is used to log values to
 * the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
 *
 * @param subsystemFor the subsystem this logger will log values for
 */
public class NetworkTableLogger {

  // Get the default instance of NetworkTables that was created automatically
  // when the robot program starts
  NetworkTableInstance inst = NetworkTableInstance.getDefault();

  // NetworkTable to log to
  NetworkTable table;

  // Publishers for each supported type
  private final ConcurrentHashMap<String, BooleanPublisher> booleanPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, DoublePublisher> doublePublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, IntegerPublisher> integerPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StringPublisher> stringPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, DoubleArrayPublisher> doubleArrayPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StructPublisher<Pose2d>> pose2dPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StructPublisher<Pose3d>> pose3dPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StructArrayPublisher<SwerveModuleState>> swerveModuleStatePublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StructPublisher<ChassisSpeeds>> chassisSpeedsPublishers = new ConcurrentHashMap<>();
  
  // -=-=- Stuff for log(key, value) =-=-=
  @SuppressWarnings("PMD.UseConcurrentHashMap")
  private static final Map<String, Sendable> tablesToData = new HashMap<>();

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  /**
   * Custom NetworkTable logger created by Glitch 2.0 in 2025. This logger is used to log values to
   * the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
   *
   * @param subsystemFor the subsystem this logger will log values for
   */
  public NetworkTableLogger(String subsystemFor) {

    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called what the parameter
    // subsystemFor holds: (the subsystem to log for).
    table = inst.getTable(subsystemFor);
  }

  public NetworkTable getNetworkTable() {
    return table;
  }

  /**
   * Log method for logging a double to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param value the value (double) that will be logged
   */
  public void logDouble(String key, double value) {
    if (!doublePublishers.containsKey(key)) {
      doublePublishers.put(key, table.getDoubleTopic(key).publish());
    }

    doublePublishers.get(key).set(value);
  }

    /**
   * Log method for logging an integer to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param value the value (int) that will be logged
   */
  public void logInt(String key, int value) {
    if (!integerPublishers.containsKey(key)) {
      integerPublishers.put(key, table.getIntegerTopic(key).publish());
    }

    integerPublishers.get(key).set(value);
  }

  /**
   * Log method for logging a boolean to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param value the value (boolean) that will be logged
   */
  public void logBoolean(String key, boolean value) {
    if (!booleanPublishers.containsKey(key)) {
      booleanPublishers.put(key, table.getBooleanTopic(key).publish());
    }

    booleanPublishers.get(key).set(value);
  }

  /**
   * Get method for retrieving a Pose2d array from the network table.
   * The underlying double array is expected to have a length that is a multiple of 3,
   * where each group of three values represents x, y, and rotation (in radians) for a Pose2d.
   *
   * @param key the key representing the Pose2d array
   * @param defaultValue the default Pose2d array to return if the retrieved array is invalid
   * @return an array of Pose2d built from the flattened double array from the network table
   */
  public Pose2d[] getPose2dArray(String key) {
      // Retrieve a flattened double array; default to empty array if not found.
      double[] doubles = table.getEntry(key).getDoubleArray(new double[0]);

      // Validate that the array has a length that is a multiple of 3.
      if (doubles.length == 0 || doubles.length % 3 != 0) {
          return new Pose2d[0];
      }

      int numPoses = doubles.length / 3;
      Pose2d[] poses = new Pose2d[numPoses];
      for (int i = 0; i < numPoses; i++) {
          double x = doubles[i * 3];
          double y = doubles[i * 3 + 1];
          double rotation = doubles[i * 3 + 2];
          poses[i] = new Pose2d(new Translation2d(x, y), new Rotation2d(rotation));
      }
      return poses;
  }
    
  /**
   * Get method for retrieving a boolean from the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that represents the value
   * @return the boolean value associated with the key
   */
  public boolean getBoolean(String key, boolean defaultValue) {
    return table.getBooleanTopic(key).getEntry(defaultValue).get();
  }

  /**
   * Log method for logging a string to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param string the value (string) that will be logged
   */
  public void logString(String key, String value) {
    if (!stringPublishers.containsKey(key)) {
      stringPublishers.put(key, table.getStringTopic(key).publish());
    }

    stringPublishers.get(key).set(value);
  }

  /**
   * Logs any object that is able to be sent over NetworkTables (Sendable) to the SmartDashboard (use for debug). Avoid using this if possible;
   * make a new method in NetworkTableLogger to log specific data type. (Can be seen using
   * AdvantageScope, Glass, Elastic, etc.)
   *
   * @apiNote Sendable: a wrapper of certain objects, fields, and methods that can be sent to NetworkTables (ex: double, int, string, their array varieties).
   * 
   * @param key the key, a string, that will represent the value in the SmartDashboard Network Table
   * @param value the NT accepted value (Sendable) to log. (This parameter can just be the bare object, field
   *     or method if it is applicable as a sendable)
   */
  public void logToSmartDash(String key, Sendable value) {
    if (!table.containsKey(key)) SmartDashboard.putData(key, value);
    SmartDashboard.updateValues();
  }

  /**
   * Log method for logging a can status to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param value the value (double) that will be logged
   */
  public void logCan(String key, CANStatus value) {
    double[] canStatusArray = new double[] {
      value.percentBusUtilization,
      value.busOffCount,
      value.txFullCount,
      value.receiveErrorCount,
      value.transmitErrorCount
    };

    if (!doubleArrayPublishers.containsKey(key)) {
      doubleArrayPublishers.put(key, table.getDoubleArrayTopic(key).publish());
    }

    doubleArrayPublishers.get(key).set(canStatusArray);
  }

  /**
   * Log method for logging a Pose2d to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param pose2d the value (Pose2d) that will be logged
   */
  public void logPose2d(String key, Pose2d pose2d) {
    if (!pose2dPublishers.containsKey(key)) {
      pose2dPublishers.put(key, table.getStructTopic(key, Pose2d.struct).publish());
    }

    pose2dPublishers.get(key).set(pose2d);
  }

  /**
   * Log method for logging a Pose3d to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param pose3d the value (Pose3d) that will be logged
   */
  public void logPose3d(String key, Pose3d pose3d) {
    if (!pose3dPublishers.containsKey(key)) {
      pose3dPublishers.put(key, table.getStructTopic(key, Pose3d.struct).publish());
    }

    pose3dPublishers.get(key).set(pose3d);
  }

  /**
   * Log method for logging the ServeModuleStates to the network table (can be seen using
   * AdvantageScope, Glass, Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param swerveModuleStateList the value (SwerveModuleState[]) that will be logged
   */
  public void logSwerveModuleState(String key, SwerveModuleState[] swerveModuleStateList) {
    if (!swerveModuleStatePublishers.containsKey(key)) {
      swerveModuleStatePublishers.put(key, table.getStructArrayTopic(key, SwerveModuleState.struct).publish());
    }

    swerveModuleStatePublishers.get(key).set(swerveModuleStateList);
  }

  /**
   * Log method for logging a ChassisSpeeds to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param chassisSpeeds the value (ChassisSpeeds) that will be logged
   */
  public void logChassisSpeeds(String key, ChassisSpeeds chassisSpeeds) {
    if (!chassisSpeedsPublishers.containsKey(key)) {
      chassisSpeedsPublishers.put(key, table.getStructTopic(key, ChassisSpeeds.struct).publish());
    }

    chassisSpeedsPublishers.get(key).set(chassisSpeeds);
  }

  /**
   * Logs a Sendable to the network table. This is useful for logging certain functions and also for
   * logging Field2d objects.
   *
   * @apiNote Sendable: a wrapper of certain objects, fields, and methods that can be sent to a
   *     network table. (This parameter can just be the bare object, field or method)
   * @param key the key, a string, that will represent the value
   * @param field2d the value (Field2d) that will be logged
   */
  @SuppressWarnings("PMD.CompareObjectsWithEquals")
  // (!table.containsKey(key))> AND <for (Sendable data :
  // tablesToData.values())> depending on if it updates
  public void logField2d(String key, Field2d field2d) {
    if (!table.containsKey(key)) {
      Sendable sddata = tablesToData.get(key);
      if (sddata == null || sddata != field2d) {
        tablesToData.put(key, field2d);
        NetworkTable dataTable = table.getSubTable(key);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable);
        SendableRegistry.publish(field2d, builder);
        builder.startListeners();
        dataTable.getEntry(".name").setString(key);
      }
    }

    for (Sendable data : tablesToData.values()) {
      SendableRegistry.update(data);
    }
  }
}
