package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Manipulator extends SubsystemBase {

    private static Manipulator MANIPULATOR;

    ManipulatorState state = ManipulatorState.UP;

    DoubleSolenoid coralSolenoid;

    DigitalInput leftButton;
    DigitalInput rightButton;

    public Manipulator(int leftButtonID, int rightButtonID) {
        coralSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

        leftButton = new DigitalInput(leftButtonID);
        rightButton = new DigitalInput(rightButtonID);
    }

    public Command dropCoral() {
        return Commands.runOnce(() -> {
            coralSolenoid.set(DoubleSolenoid.Value.kReverse);
            state = ManipulatorState.DOWN;
        });
    }

    public Command returnCoral() {
        return Commands.runOnce(() -> {
            coralSolenoid.set(DoubleSolenoid.Value.kForward);
            state = ManipulatorState.UP;
        });
    }

    public SequentialCommandGroup dropCoralAndReturn() {
        return new SequentialCommandGroup(dropCoral(), new WaitCommand(1), returnCoral());
    }

    public String getManipualtorState() {
        return state.toString();
    }

    public boolean getLeftButtonState() {
        return leftButton.get();
    }

    public boolean getRightButtonState() {
        return leftButton.get();
    }

    public static Manipulator getInstance() {
        if (MANIPULATOR == null) {
            MANIPULATOR = new Manipulator(4, 5);
        }
        return MANIPULATOR;
    }

    private enum ManipulatorState {
        UP,
        DOWN,

    }
}
