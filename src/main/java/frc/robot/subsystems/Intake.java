package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    IntakeState state = IntakeState.EMPTY;

    public void init() {}

    public IntakeState intake() {
        if (state != IntakeState.EMPTY) {
            return IntakeState.ERROR;
        }

        //TODO intake
        state = IntakeState.FULL;
        return state;
    }

    public String getIntakeState() {
        return state.toString();
    }

    private enum IntakeState {
        EMPTY,
        FULL,
        ERROR,
    }
}
