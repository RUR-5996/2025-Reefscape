package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
    private final Solenoid left_intake_solenoid_1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid left_intake_solenoid_2 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid right_intake_solenoid_1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid right_intake_solenoid_2 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    private final Solenoid elevator_solenoid_1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid elevator_solenoid_2 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid elevator_solenoid_3 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    private final Solenoid climber_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);


    public void toggleLeftIntake() {
        left_intake_solenoid_1.set(!left_intake_solenoid_1.get());
        left_intake_solenoid_1.set(!left_intake_solenoid_2.get());
    }

    public Command leftIntake() {
        return Commands.runOnce(
            () -> {
                toggleLeftIntake();
            }
        );
    }

    public void toggleRightIntake() {
        right_intake_solenoid_1.set(!right_intake_solenoid_1.get());
        right_intake_solenoid_1.set(!right_intake_solenoid_2.get());
    }

    public Command rightIntake() {
        return Commands.runOnce(
            () -> {
                toggleRightIntake();
            }
        );
    }

    public void toggle_elevator_1() {
        elevator_solenoid_1.set(!elevator_solenoid_1.get());
    }

    public Command Elevator1() {
        return Commands.runOnce(
            () -> {
                toggle_elevator_1();
            }
        );
    }

    public void toggle_elevator_2() {
        elevator_solenoid_2.set(!elevator_solenoid_1.get());
    }


    public Command Elevator2() {
        return Commands.runOnce(
            () -> {
                toggle_elevator_2();
            }
        );
    }

    public void toggle_elevator_3() {
        elevator_solenoid_3.set(!elevator_solenoid_1.get());
    }


    public Command Elevator3() {
        return Commands.runOnce(
            () -> {
                toggle_elevator_3();
            }
        );
    }

    public void toggle_climber() {
        climber_solenoid.set(!climber_solenoid.get());
    }


    public Command toggleClimber() {
        return Commands.runOnce(
            () -> {
                toggle_climber();
            }
        );
    }

}
