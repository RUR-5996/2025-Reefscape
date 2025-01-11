package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    ElevatorState state = ElevatorState.DOWN;

    public void init() {}

    public ElevatorState elevate(ElevatorState floor) {
        if (state == floor) { //if already on correct floor, return
            return floor;
        }
        //TODO elevator
        return state;
    }

    public String getElevatorState() {
        return state.toString();
    }

    private enum ElevatorState {
        DOWN,
        FLOOR0,
        FLOOR1,
        FLOOR2,
        FLOOR3,
    }
}
