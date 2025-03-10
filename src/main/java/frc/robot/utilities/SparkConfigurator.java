package frc.robot.utilities;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import java.util.Set;

public class SparkConfigurator {
  // Frame speeds in ms
  private static final int FAST = 10;
  private static final int NORMAL = 20;
  private static final int SLOW = 200;
  private static final int OFF = 65535;

  // Sensor options
  public enum Sensors {
    INTEGRATED,
    ABSOLUTE,
    ALTERNATE,
    ANALOG
  }

  // Data logging options
  public enum LogData {
    VOLTAGE,
    CURRENT,
    POSITION,
    VELOCITY
  }

  // Get a sparkmax with no follower, sensors, or logged data
  public static SparkMax getSparkMax(int id, MotorType motorType) {
    return getSparkMax(id, motorType, false, Set.of(), Set.of());
  }

  public static SparkFlex getSparkFlex(int id, MotorType motorType) {
    return getSparkFlex(id, motorType, false, Set.of(), Set.of());
  }

  // Get a sparkmax with no sensors or logged data
  public static SparkMax getSparkMax(int id, MotorType motorType, boolean hasFollower) {
    return getSparkMax(id, motorType, hasFollower, Set.of(), Set.of());
  }

  public static SparkFlex getSparkFlex(int id, MotorType motorType, boolean hasFollower) {
    return getSparkFlex(id, motorType, hasFollower, Set.of(), Set.of());
  }

  // Get a sparkmax
  public static SparkMax getSparkMax(
      int id,
      MotorType motorType,
      boolean hasFollower,
      Set<Sensors> sensors,
      Set<LogData> logData) {

    // NEW 2025 CREATION OF SPARKMAX, CANSPARKMAX was removed
    SparkMax spark = new SparkMax(id, motorType);
    SparkMaxConfig config = new SparkMaxConfig();
    // spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
    // TODO: this line might need to be removed. It wasn't there
    // TODO: i think it needs to be removed too

    // before, but I added it to set the spark to an empty config
    // (factory reset)
    // spark.restoreFactoryDefaults();

    int[] status = {FAST, SLOW, SLOW, OFF, OFF, OFF, OFF};
    // status0 Applied Output & Faults
    // status1 Velocity, Voltage, & Current
    // status2 Position
    // status3 Analog Sensor
    // status4 Alternate Encoder
    // status5 Absolute Encoder Position
    // status6 Absolute Encoder Velocity

    if (!hasFollower && !logData.contains(LogData.VOLTAGE)) {
      status[0] = SLOW;
    }

    if (logData.contains(LogData.VELOCITY)
        || logData.contains(LogData.VOLTAGE)
        || logData.contains(LogData.CURRENT)) {
      status[1] = NORMAL;
    }

    if (logData.contains(LogData.POSITION)) status[2] = NORMAL;

    if (sensors.contains(Sensors.ANALOG)) status[3] = NORMAL;

    if (sensors.contains(Sensors.ALTERNATE)) status[4] = NORMAL;

    if (sensors.contains(Sensors.ABSOLUTE)) {
      if (logData.contains(LogData.POSITION)) status[5] = NORMAL;
      if (logData.contains(LogData.VELOCITY)) status[6] = NORMAL;
    }

    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < Constants.configurationSetRetries; j++) {

        // NEW FOR 2025
        switch (i) {
          case 0:
            config.signals.appliedOutputPeriodMs(status[i]); // Applied Output
            config.signals.faultsPeriodMs(status[i]); // All faults logging
            config.signals.busVoltagePeriodMs(status[i]);
            config.signals.outputCurrentPeriodMs(status[i]);
            config.signals.motorTemperaturePeriodMs(status[i]);
          case 1:
            // TODO: Add motor velocity, voltage and current periods (research if still exists)
            config.signals.primaryEncoderVelocityPeriodMs(status[i]);
          case 2:
            // TODO: Add motor position period (research if still exists)
            config.signals.primaryEncoderPositionPeriodMs(status[i]);
          case 3:
            config.signals.analogVoltagePeriodMs(status[i]);
            config.signals.analogVelocityPeriodMs(status[i]);
            config.signals.analogPositionPeriodMs(status[i]);
          case 4:
            config.signals.externalOrAltEncoderVelocity(status[i]);
            config.signals.externalOrAltEncoderPosition(status[i]);
          case 5:
            config.signals.absoluteEncoderPositionPeriodMs(status[i]);
            // Absolute (duty cycle) encoder angle?
          case 6:
            config.signals.absoluteEncoderVelocityPeriodMs(status[i]);
            // Absolute (duty cycle) encoder frequency?
        }
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // OLD//spark.setPeriodicFramePeriod(PeriodicFrame.values()[i], status[i]);

      }
    }

    return spark;
  }

  // Get a sparkflex
  public static SparkFlex getSparkFlex(
      int id,
      MotorType motorType,
      boolean hasFollower,
      Set<Sensors> sensors,
      Set<LogData> logData) {

    // NEW 2025 CREATION OF SPARKMAX, CANSPARKMAX was removed
    SparkFlex spark = new SparkFlex(id, motorType);
    SparkFlexConfig config = new SparkFlexConfig();
    spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode
            .kPersistParameters); // TODO: this line might need to be removed. It wasn't there
    // before, but I added it to set the spark to an empty config
    // (factory reset)
    // spark.restoreFactoryDefaults();

    int[] status = {FAST, SLOW, SLOW, OFF, OFF, OFF, OFF};
    // status0 Applied Output & Faults
    // status1 Velocity, Voltage, & Current
    // status2 Position
    // status3 Analog Sensor
    // status4 Alternate Encoder
    // status5 Absolute Encoder Position
    // status6 Absolute Encoder Velocity

    if (!hasFollower && !logData.contains(LogData.VOLTAGE)) {
      status[0] = SLOW;
    }

    if (logData.contains(LogData.VELOCITY)
        || logData.contains(LogData.VOLTAGE)
        || logData.contains(LogData.CURRENT)) {
      status[1] = NORMAL;
    }

    if (logData.contains(LogData.POSITION)) status[2] = NORMAL;

    if (sensors.contains(Sensors.ANALOG)) status[3] = NORMAL;

    if (sensors.contains(Sensors.ALTERNATE)) status[4] = NORMAL;

    if (sensors.contains(Sensors.ABSOLUTE)) {
      if (logData.contains(LogData.POSITION)) status[5] = NORMAL;
      if (logData.contains(LogData.VELOCITY)) status[6] = NORMAL;
    }

    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < Constants.configurationSetRetries; j++) {

        // NEW FOR 2025
        switch (i) {
          case 0:
            config.signals.appliedOutputPeriodMs(status[i]); // Applied Output
            config.signals.faultsPeriodMs(status[i]); // All faults logging
            config.signals.busVoltagePeriodMs(status[i]);
            config.signals.outputCurrentPeriodMs(status[i]);
            config.signals.motorTemperaturePeriodMs(status[i]);
          case 1:
            // TODO: Add motor velocity, voltage and current periods (research if still exists)
            config.signals.primaryEncoderVelocityPeriodMs(status[i]);
          case 2:
            // TODO: Add motor position period (research if still exists)
            config.signals.primaryEncoderPositionPeriodMs(status[i]);
          case 3:
            config.signals.analogVoltagePeriodMs(status[i]);
            config.signals.analogVelocityPeriodMs(status[i]);
            config.signals.analogPositionPeriodMs(status[i]);
          case 4:
            config.signals.externalOrAltEncoderVelocity(status[i]);
            config.signals.externalOrAltEncoderPosition(status[i]);
          case 5:
            config.signals.absoluteEncoderPositionPeriodMs(status[i]);
            // Absolute (duty cycle) encoder angle?
          case 6:
            config.signals.absoluteEncoderVelocityPeriodMs(status[i]);
            // Absolute (duty cycle) encoder frequency?
        }
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // OLD//spark.setPeriodicFramePeriod(PeriodicFrame.values()[i], status[i]);

      }
    }

    return spark;
  }

  public static SparkMax getFollowerMax(
      SparkMax leader, int id, MotorType motorType, boolean invert) {

    // NEW FOR 2025, CANSPARKMAX WAS REMOVED, NOW SPARKMAX is used and TO CONFIGURE A SPARK MAX WE
    // HAVE TO MAKE A SPARKMAXCONFIG OBJECT
    SparkMax spark = new SparkMax(id, motorType);
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(leader, invert);
    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // spark.follow(leader, invert);

    int[] status = {SLOW, SLOW, SLOW, OFF, OFF, OFF, OFF};
    // status0 Applied Output & Faults
    // status1 Velocity, Voltage, & Current
    // status2 Position
    // status3 Analog Sensor
    // status4 Alternate Encoder
    // status5 Absolute Encoder Position
    // status6 Absolute Encoder Velocity

    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < Constants.configurationSetRetries; j++) {
        // NEW FOR 2025
        switch (i) {
          case 0:
            config.signals.appliedOutputPeriodMs(status[i]); // Applied Output
            config.signals.faultsPeriodMs(status[i]); // All faults logging
            config.signals.busVoltagePeriodMs(status[i]);
            config.signals.outputCurrentPeriodMs(status[i]);
            config.signals.motorTemperaturePeriodMs(status[i]);
          case 1:
            // TODO: Add motor velocity, voltage and current periods (research if still exists)
            config.signals.primaryEncoderVelocityPeriodMs(status[i]);
          case 2:
            // TODO: Add motor position period (research if still exists)
            config.signals.primaryEncoderPositionPeriodMs(status[i]);
          case 3:
            config.signals.analogVoltagePeriodMs(status[i]);
            config.signals.analogVelocityPeriodMs(status[i]);
            config.signals.analogPositionPeriodMs(status[i]);
          case 4:
            config.signals.externalOrAltEncoderVelocity(status[i]);
            config.signals.externalOrAltEncoderPosition(status[i]);
          case 5:
            config.signals.absoluteEncoderPositionPeriodMs(status[i]);
            // Absolute (duty cycle) encoder angle?
          case 6:
            config.signals.absoluteEncoderVelocityPeriodMs(status[i]);
            // Absolute (duty cycle) encoder frequency?
        }
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // OLD//spark.setPeriodicFramePeriod(PeriodicFrame.values()[i], status[i]);

        try {
          Thread.sleep(5);
        } catch (Exception e) {
        }
      }
    }

    return spark;
  }

  public static SparkFlex getFollowerFlex(
      SparkFlex leader, int id, MotorType motorType, boolean invert) {

    // NEW FOR 2025, CANSPARKMAX WAS REMOVED, NOW SPARKMAX (SparkFlex) is used and TO CONFIGURE A
    // SPARK MAX (SparkFlex) WE HAVE TO MAKE A SPARKMAXCONFIG (SparkFlexConfig) OBJECT
    SparkFlex spark = new SparkFlex(id, motorType);
    SparkFlexConfig config = new SparkFlexConfig();
    config.follow(leader, invert);
    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    // spark.follow(leader, invert);

    int[] status = {SLOW, SLOW, SLOW, OFF, OFF, OFF, OFF};
    // status0 Applied Output & Faults
    // status1 Velocity, Voltage, & Current
    // status2 Position
    // status3 Analog Sensor
    // status4 Alternate Encoder
    // status5 Absolute Encoder Position
    // status6 Absolute Encoder Velocity

    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < Constants.configurationSetRetries; j++) {

        // NEW FOR 2025
        switch (i) {
          case 0:
            config.signals.appliedOutputPeriodMs(status[i]); // Applied Output
            config.signals.faultsPeriodMs(status[i]); // All faults logging
            config.signals.busVoltagePeriodMs(status[i]);
            config.signals.outputCurrentPeriodMs(status[i]);
            config.signals.motorTemperaturePeriodMs(status[i]);
          case 1:
            // TODO: Add motor velocity, voltage and current periods (research if still exists)
            config.signals.primaryEncoderVelocityPeriodMs(status[i]);
          case 2:
            // TODO: Add motor position period (research if still exists)
            config.signals.primaryEncoderPositionPeriodMs(status[i]);
          case 3:
            config.signals.analogVoltagePeriodMs(status[i]);
            config.signals.analogVelocityPeriodMs(status[i]);
            config.signals.analogPositionPeriodMs(status[i]);
          case 4:
            config.signals.externalOrAltEncoderVelocity(status[i]);
            config.signals.externalOrAltEncoderPosition(status[i]);
          case 5:
            config.signals.absoluteEncoderPositionPeriodMs(status[i]);
            // Absolute (duty cycle) encoder angle?
          case 6:
            config.signals.absoluteEncoderVelocityPeriodMs(status[i]);
            // Absolute (duty cycle) encoder frequency?
        }
        spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // OLD//spark.setPeriodicFramePeriod(PeriodicFrame.values()[i], status[i]);

        try {
          Thread.sleep(5);
        } catch (Exception e) {
        }
      }
    }

    return spark;
  }
}
