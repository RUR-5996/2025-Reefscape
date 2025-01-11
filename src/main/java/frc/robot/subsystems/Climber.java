package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    ClimberState state = ClimberState.IDLE;

    public void init() {}

    public ClimberState climb(ClimberState desiredState) {
        if (state != ClimberState.IDLE) {
            return ClimberState.ERROR;
        }
        if (desiredState == ClimberState.SHALLOW) {
            //TODO climb
        } else if (desiredState == ClimberState.DEEP) {
            //TODO climb
        }
        return desiredState; //remove after method is done?
    }

    public String getClimberState() {
        return state.toString();
    }

    public void Climb() {}

    private enum ClimberState {
        IDLE,
        SHALLOW,
        DEEP,
        ERROR,
    }
}
