package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    Spark blinkin = new Spark(0);
    private double colour = Constants.ColourConstants.PINK;

    private static LEDs LEDS;
    
    public LEDs() {

    }

    @Override
    public void periodic() {
        blinkin.set(colour);
    }

    public static LEDs getInstance() {
        if(LEDS == null) {
            LEDS = new LEDs();
        }
        return LEDS;
    }

    public Command setColour(double constant) {
        return Commands.runOnce(
            () -> {
                colour = constant;
            }
        );
    }
}
