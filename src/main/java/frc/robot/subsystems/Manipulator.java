package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {

    ManipulatorState state = ManipulatorState.EMPTY;

    public void init() {}

    public ManipulatorState pickUp() {
        if (state == ManipulatorState.FULL) {
            return ManipulatorState.ERROR;
        }
        //TODO pick up
        state = ManipulatorState.FULL;
        return state;
    }

    public ManipulatorState dropOff() {
        if (state == ManipulatorState.EMPTY) {
            return ManipulatorState.ERROR;
        }
        //TODO drop off
        state = ManipulatorState.EMPTY;
        return state;
    }

    public String getManipualtorState() {
        return state.toString();
    }

    private enum ManipulatorState {
        EMPTY,
        FULL,
        ERROR,
    }
}
