package frc.robot.utilities;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableLogger {

    DoublePublisher publisher;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    String subsystemFor;
    NetworkTable table;

    public NetworkTableLogger(String subsystemFor) {
        
        this.subsystemFor = subsystemFor;
        table = inst.getTable(subsystemFor);
    }

    public void logDouble(String key, double value) {

        if (!table.containsKey(key)) {
            publisher = table.getDoubleTopic(key).publish();
        }

        publisher.set(value);
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

}
