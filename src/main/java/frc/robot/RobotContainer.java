package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.LEDs;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

public class RobotContainer {

  private final CommandXboxController xBox = new CommandXboxController(0);

          static DigitalInput diginLeftBasket1 = new DigitalInput(0);
          static DigitalInput diginLeftBasket2 = new DigitalInput(1);
          static DigitalInput diginRightBasket1 = new DigitalInput(2);
          static DigitalInput diginRightBasket2 = new DigitalInput(3);

          private LEDs LEDS;

          public RobotContainer() {
            LEDS = LEDs.getInstance();

            configureBindings();
          
      }

      public void autoChangeRequest(String auto1, String auto2) {
          SmartDashboard.putData(LEDS);
      }

      private void configureBindings() {
        //xBox.b().toggleOnTrue(SWERVE.toggleSlowMode());

        xBox.a().onTrue(LEDS.changeColour());
        // xBox.x().onTrue(PNEUMATICS.rightIntake());
        // xBox.y().onTrue(PNEUMATICS.Elevator1());
        // xBox.rightBumper().onTrue(PNEUMATICS.Elevator2());
        // xBox.leftBumper().onTrue(PNEUMATICS.Elevator3());
        // xBox.x().onTrue(PNEUMATICS.toggleClimber());

        // xBox.a().onTrue(ELEVATOR.elevate(frc.robot.subsystems.Elevator.ElevatorState.DOWN));
        // xBox.b().onTrue(ELEVATOR.elevate(frc.robot.subsystems.Elevator.ElevatorState.FLOOR1));
        // xBox.x().onTrue(ELEVATOR.elevate(frc.robot.subsystems.Elevator.ElevatorState.FLOOR2));
        // xBox.y().onTrue(ELEVATOR.elevate(frc.robot.subsystems.Elevator.ElevatorState.FLOOR3));


      }

      private void loadPaths() {
      }

      public void periodic() {
      }

      /*public static void check_for_auto_change_periodic() {
        if (diginLeftBasket1.get() && diginLeftBasket2.get()) {
          //Robot.m_autonomousCommand = "autopath"; //idk co to ma delat
          Robot.m_autonomousCommand.schedule();
    }

    if (diginRightBasket1.get() && diginRightBasket2.get()) {

    }
  }*/

}
