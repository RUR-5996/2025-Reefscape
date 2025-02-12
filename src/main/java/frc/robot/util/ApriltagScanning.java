package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public final class ApriltagScanning {
    private static ApriltagScanning APRIL;

    public static ApriltagScanning getInstance() {
        if(APRIL == null) {
            APRIL = new ApriltagScanning();
        }
        return APRIL;
    }
    //todo
    public void periodic() {
        if (Vision.april_periodic() /*&& holding game piece*/ ) {             //detected april tag
            //begin pathing to reef detection position
            //update reef data structure
            //decide on where to place a game piece
            //path to correct location
        }
    }
}
