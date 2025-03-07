package frc.robot.ui;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.constants.UIConstants;
import frc.robot.util.Elastic;

public class CoralPositionSelector {
    private final SendableChooser<String> positionChooser = new SendableChooser<>();
    
    public CoralPositionSelector(String tabName) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        
        for (String position : UIConstants.CoralPositionUI.POSITIONS) {
            positionChooser.addOption(position, position);
        }
        positionChooser.setDefaultOption(UIConstants.CoralPositionUI.POSITIONS[0], UIConstants.CoralPositionUI.POSITIONS[0]);
        
        tab.add(UIConstants.CoralPositionUI.POSITION_KEY, positionChooser)
           .withSize((int)UIConstants.CoralPositionUI.SELECTOR_WIDTH, (int)UIConstants.CoralPositionUI.SELECTOR_HEIGHT)
           .withPosition(0, 1);
    }

    public void periodic() {
        String selected = positionChooser.getSelected();
        if (selected != null) {
            Elastic.setCoralPosition(selected);
        }
    }
}
