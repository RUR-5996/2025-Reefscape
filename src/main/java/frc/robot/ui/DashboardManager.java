package frc.robot.ui;


import edu.wpi.first.wpilibj.DriverStation;

public class DashboardManager {
    private static final String AUTO_TAB_NAME = "Auto";

    // TODO rewrite to Smartdashboard

    private final StartPositionSelector startSelector;
    private final CoralPositionSelector coralSelector;
    public final ReefSidePicker reefSidePicker;

    public DashboardManager() {
        startSelector = new StartPositionSelector(AUTO_TAB_NAME);
        coralSelector = new CoralPositionSelector(AUTO_TAB_NAME);
        reefSidePicker = new ReefSidePicker(AUTO_TAB_NAME, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)); // default to blue alliacne for testing purposes
    }
    
    public void periodic() {
        startSelector.periodic();
        coralSelector.periodic();
        reefSidePicker.periodic();
    }
}
