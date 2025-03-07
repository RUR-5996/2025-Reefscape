package frc.robot.ui;

public class DashboardManager {
    private static final String AUTO_TAB_NAME = "Auto";
    
    private final StartPositionSelector startSelector;
    private final CoralPositionSelector coralSelector;
    
    public DashboardManager() {
        startSelector = new StartPositionSelector(AUTO_TAB_NAME);
        coralSelector = new CoralPositionSelector(AUTO_TAB_NAME);
    }
    
    public void periodic() {
        startSelector.periodic();
        coralSelector.periodic();
    }
}
