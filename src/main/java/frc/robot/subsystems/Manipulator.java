package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Manipulator extends SubsystemBase {

    private static Manipulator MANIPULATOR;

    ManipulatorState state = ManipulatorState.UP;

    Talon coralMotor;
    Solenoid coralSolenoid;


    public Manipulator() {
        coralMotor = new Talon(10);
        coralSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    }

    public Command dropCoral() {
        return Commands.runOnce(() -> {
            coralSolenoid.set(false);
            coralMotor.set(1);
            state = ManipulatorState.DOWN;
        });
    }

    public Command returnCoral() {
        return Commands.runOnce(() -> {
            coralSolenoid.set(true);
            coralMotor.set(0);
            state = ManipulatorState.UP;
        });
    }

    public SequentialCommandGroup dropCoralAndReturn() {
        return new SequentialCommandGroup(dropCoral(), new WaitCommand(1), returnCoral());
    }

    public String getManipualtorState() {
        return state.toString();
    }

    public static Manipulator getInstance() {
        if (MANIPULATOR == null) {
            MANIPULATOR = new Manipulator();
        }
        return MANIPULATOR;
    }

    private enum ManipulatorState {
        UP,
        DOWN,
        ERROR,
    }
}
