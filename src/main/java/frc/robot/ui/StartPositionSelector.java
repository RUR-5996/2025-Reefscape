package frc.robot.ui;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.util.Elastic;

public class StartPositionSelector {
    private final SendableChooser<String> positionChooser = new SendableChooser<>();

    public StartPositionSelector(String tabName) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);

        for (String position : Constants.UIConstants.StartPositionUI.POSITIONS) {
            positionChooser.addOption(position, position);
        }
        positionChooser.setDefaultOption(Constants.UIConstants.StartPositionUI.POSITIONS[0], Constants.UIConstants.StartPositionUI.POSITIONS[0]);

        tab.add(Constants.UIConstants.StartPositionUI.POSITION_KEY, positionChooser)
           .withSize((int)Constants.UIConstants.StartPositionUI.SELECTOR_WIDTH, (int)Constants.UIConstants.StartPositionUI.SELECTOR_HEIGHT)
           .withPosition(0, 0);
    }

    public void periodic() {
        String selected = positionChooser.getSelected();
        if (selected != null) {
            Elastic.setStartPosition(selected);
        }
    }
}
