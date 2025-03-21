package frc.robot.ui;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.util.Optional;

public class ReefSidePicker {
    // upper right, center right, lower right, lower left, center left, upper left

    Integer[] ids;
    Integer selected = 0;
    public Integer manualReefID;

    public ReefSidePicker(String tabName, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            ids = Constants.UIConstants.reefSideUI.redTags;
        }
        else {
            ids = Constants.UIConstants.reefSideUI.blueTags;
        }
        SmartDashboard.putBoolean(ids[0].toString(), true); // TODO layout setup + downloading from robot?
        SmartDashboard.putBoolean(ids[1].toString(), false);
        SmartDashboard.putBoolean(ids[2].toString(), false);
        SmartDashboard.putBoolean(ids[3].toString(), false);
        SmartDashboard.putBoolean(ids[4].toString(), false);
        SmartDashboard.putBoolean(ids[5].toString(), false);
    }


    public void periodic() {
        if (SmartDashboard.getBoolean(ids[0].toString(), false) && !(selected  == 0)) {
            SmartDashboard.putBoolean(ids[selected].toString(), false);
            selected = 0;
        } else if (SmartDashboard.getBoolean(ids[1].toString(), false) && !(selected  == 1)) {
            SmartDashboard.putBoolean(ids[selected].toString(), false);
            selected = 1;
        } else if (SmartDashboard.getBoolean(ids[2].toString(), false) && !(selected  == 2)) {
            SmartDashboard.putBoolean(ids[selected].toString(), false);
            selected = 2;
        } else if (SmartDashboard.getBoolean(ids[3].toString(), false) && !(selected  == 3)) {
            SmartDashboard.putBoolean(ids[selected].toString(), false);
            selected = 3;
        } else if (SmartDashboard.getBoolean(ids[4].toString(), false) && !(selected  == 4)) {
            SmartDashboard.putBoolean(ids[selected].toString(), false);
            selected = 4;
        } else if (SmartDashboard.getBoolean(ids[5].toString(), false) && !(selected  == 5)) {
            SmartDashboard.putBoolean(ids[selected].toString(), false);
            selected = 5;
        }
        manualReefID = ids[selected];
    }
}
