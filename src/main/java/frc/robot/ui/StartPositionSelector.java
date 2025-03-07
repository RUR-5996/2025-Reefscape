package frc.robot.ui;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.constants.UIConstants;
import frc.robot.util.Elastic;

public class StartPositionSelector {
    private final SendableChooser<String> positionChooser = new SendableChooser<>();
    
    public StartPositionSelector(String tabName) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        
        for (String position : UIConstants.StartPositionUI.POSITIONS) {
            positionChooser.addOption(position, position);
        }
        positionChooser.setDefaultOption(UIConstants.StartPositionUI.POSITIONS[0], UIConstants.StartPositionUI.POSITIONS[0]);
        
        tab.add(UIConstants.StartPositionUI.POSITION_KEY, positionChooser)
           .withSize((int)UIConstants.StartPositionUI.SELECTOR_WIDTH, (int)UIConstants.StartPositionUI.SELECTOR_HEIGHT)
           .withPosition(0, 0);
    }

    public void periodic() {
        String selected = positionChooser.getSelected();
        if (selected != null) {
            Elastic.setStartPosition(selected);
        }
    }
}
