package frc.robot.ui;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.util.Elastic;

public class CoralPositionSelector {
    private final SendableChooser<String> positionChooser = new SendableChooser<>();

    public CoralPositionSelector(String tabName) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        for (String position : Constants.UIConstants.CoralPositionUI.POSITIONS) {
            positionChooser.addOption(position, position);
        }
        positionChooser.setDefaultOption(Constants.UIConstants.CoralPositionUI.POSITIONS[0], Constants.UIConstants.CoralPositionUI.POSITIONS[0]);

        tab.add(Constants.UIConstants.CoralPositionUI.POSITION_KEY, positionChooser)
           .withSize((int)Constants.UIConstants.CoralPositionUI.SELECTOR_WIDTH, (int)Constants.UIConstants.CoralPositionUI.SELECTOR_HEIGHT)
           .withPosition(0, 1);
    }

    public void periodic() {
        String selected = positionChooser.getSelected();
        if (selected != null) {
            Elastic.setCoralPosition(selected);
        }
    }
}
