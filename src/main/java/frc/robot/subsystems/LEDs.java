package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;

public class LEDs extends SubsystemBase {
    Spark blinkin = new Spark(0);
    private double colour = Constants.ColourConstants.PINK;

    private static LEDs LEDS;

    public LEDs() {

             public final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
          

            public final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
          
            // Our LED strip has a density of 120 LEDs per meter
          
            public static final Distance kLedSpacing = Meters.of(1 / 120.0);
          
          
            // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed of 1 meter per second.

      
            m_led = new AddressableLED(1);
            // Length is expensive to set, so only set it once, then just update data

            m_ledBuffer = new AddressableLEDBuffer(300);

            m_led.setLength(m_ledBuffer.getLength());

            m_led.setData(m_ledBuffer);

            m_led.start();
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

    public Command setColour() {
        // Update the buffer with the rainbow animation
        m_scrollingRainbow.applyTo(m_ledBuffer);
        // Set the LEDs
        m_led.setData(m_ledBuffer);
            }
        );
    }
}
