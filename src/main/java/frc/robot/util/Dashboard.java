package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Dashboard {
    private static final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private static final NetworkTable dashboardTable = ntInstance.getTable("ElasticDashboard");

    private static final NetworkTableEntry selectedRowEntry = dashboardTable.getEntry("selectedRow");
    private static final NetworkTableEntry selectedColEntry = dashboardTable.getEntry("selectedCol");

    private static int selectedRow = -1;
    private static int selectedCol = -1;

    public static void selectPosition(int row, int col) {
        selectedRow = row;
        selectedCol = col;

        // Send data to Elastic/Kibana
        selectedRowEntry.setInteger(row);
        selectedColEntry.setInteger(col);
    }

    public static int getSelectedRow() {
        return (int) selectedRowEntry.getInteger(-1);
    }

    public static int getSelectedCol() {
        return (int) selectedColEntry.getInteger(-1);
    }
}
