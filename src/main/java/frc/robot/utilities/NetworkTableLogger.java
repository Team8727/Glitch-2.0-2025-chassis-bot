package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.sendable.Sendable;

/**
 * Custom NetworkTable logger created by Glitch 2.0 in 2025. 
 * This logger is used to log values to the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
 * 
 * @param subsystemFor the subsystem this logger will log values for
 */
public class NetworkTableLogger {

    // Get the default instance of NetworkTables that was created automatically
    // when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // NetworkTable to log to
    NetworkTable table;

    // Declare the publishers (type) for the commonly used network table data types
    DoublePublisher doublePublisher;
    BooleanPublisher booleanPublisher;
    StringPublisher stringPublisher;

    // Pose2d logging objects
    StructPublisher<Pose2d> pose2dPublisher;
    StructArrayPublisher<Pose2d> pose2dArrayPublisher;

    // Pose 3d logging objects
    StructPublisher<Pose3d> pose3dPublisher;
    StructArrayPublisher<Pose3d> pose3dArrayPublisher;

    //Swerve Module States logging objects
    StructArrayPublisher<SwerveModuleState> swerveModuleStatePublisher;

    //-=-=- Stuff for log(key, value) =-=-=
    @SuppressWarnings("PMD.UseConcurrentHashMap")
    private static final Map<String, Sendable> tablesToData = new HashMap<>();
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    /**
     * Custom NetworkTable logger created by Glitch 2.0 in 2025. 
     * This logger is used to log values to the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
     * 
     * @param subsystemFor the subsystem this logger will log values for
     */
    // Constructor for NetworkTableLogger that takes a subsystem name and creates a network table for the subsystem
    public NetworkTableLogger(String subsystemFor) {

        // Get the table within that instance that contains the data. There can
        // be as many tables as you like and exist to make it easier to organize
        // your data. In this case, it's a table called datatable.
        table = inst.getTable(subsystemFor);
    }


    /**
     * Log method for logging a double to the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
     * 
     * @param key the key, a string, that will represent the value
     * @param value the value (double) that will be logged
     */
    // Log methods for logging a double to the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
    public void logDouble(String key, double value) {
        if (!table.containsKey(key)) doublePublisher = table.getDoubleTopic(key).publish();
        doublePublisher.set(value);
    }

    /**
     * Log method for logging a boolean to the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
     * 
     * @param key the key, a string, that will represent the value
     * @param value the value (boolean) that will be logged
     */
    // Log methods for logging a boolean to the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
    public void logBoolean(String key, boolean value) {
        if (!table.containsKey(key)) booleanPublisher = table.getBooleanTopic(key).publish();
        booleanPublisher.set(value);
    }

    /**
     * Log method for logging a string to the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
     * 
     * @param key the key, a string, that will represent the value
     * @param string the value (string) that will be logged
     */
    // Log methods for logging a string to the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
    public void logString(String key, String string) {
        if (!table.containsKey(key)) stringPublisher = table.getStringTopic(key).publish();
        stringPublisher.set(string);
    }

    /* //Use logFunction(key, value) instead
    public void logField2d_SmartDashboard(String key, Field2d field2d) {
        if (!table.containsKey(key)) stringPublisher = table.getStringTopic(key).publish();
        SmartDashboard.putData(key, field2d);
    }
    */
    
    // Logs any value to the network table
    public void logFn_SmartDash(String key, Sendable value) {
        if (!table.containsKey(key)) SmartDashboard.putData(key, value);
        SmartDashboard.updateValues();
    }

    public void logPose2d(String key, Pose2d pose2d) {
        if (!table.containsKey(key)) pose2dPublisher = table.getStructTopic("2DPose", Pose2d.struct).publish();
        pose2dPublisher.set(pose2d);
    }

    public void logPose3d(String key, Pose3d pose3d) {
        if (!table.containsKey(key)) pose3dPublisher = table.getStructTopic("3DPose", Pose3d.struct).publish();
        pose3dPublisher.set(pose3d);
    }

    public void logSwerveModuleState(String key, SwerveModuleState[] swerveState) {
        if (!table.containsKey(key)) swerveModuleStatePublisher = table.getStructArrayTopic("swerveModuleStates", SwerveModuleState.struct).publish();
        swerveModuleStatePublisher.set(swerveState);
    }

    /**
     * Logs a Sendable to the network table.
     * This is useful for logging certain functions and also for logging Field2d objects.
     * 
     * @apiNote Sendable: a wrapper of certain objects, fields, and methods that can be sent to a network table.
     * (This parameter can just be the bare object, field or method)
     * 
     * @param key the key, a string, that will represent the value
     * @param field2d the value that will be logged 
     */
    @SuppressWarnings("PMD.CompareObjectsWithEquals") //TODO Might need to get rid of <if (!table.containsKey(key))> AND <for (Sendable data : tablesToData.values())> depending on if it updates
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
    

           /*
        //________________USING NETWORK TABLES TO PUBLISH ROBOT HEADING UNDER A TABLE CALLED "SwerveSubsystemLiveLogging" ___________
        //                SEE PERIODIC METHOD FOR THE UPDATING OF THE DOUBLETOPIC "robotHeading" USING getRobotHeading() 
            
            // Get the default instance of NetworkTables that was created automatically
            // when the robot program starts
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            // Get the table within that instance that contains the data. There can
            // be as many tables as you like and exist to make it easier to organize
            // your data. In this case, it's a table called datatable.
            NetworkTable table = inst.getTable("SwerveSubsystemLiveLogging");

            robotHeadingPublisher = table.getDoubleTopic("robotHeading").publish();

        //___________________________________________________________________________________________________________________________
        */
