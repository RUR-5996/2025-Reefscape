package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

public class RobotContainer {

  private final CommandXboxController xBox = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  private static SendableChooser<Command> autoBranchChooser;

          static DigitalInput diginLeftBasket1 = new DigitalInput(0);
          static DigitalInput diginLeftBasket2 = new DigitalInput(1);
          static DigitalInput diginRightBasket1 = new DigitalInput(2);
          static DigitalInput diginRightBasket2 = new DigitalInput(3);

          public Elevator ELEVATOR;
          public Intake INTAKE;
          public SwerveDrive SWERVE;
          public DriveTrain DRIVETRAIN;
          private LEDs LEDS;
          RobotConfig config;
          public Pneumatics PNEUMATICS;
          public Vision VISION;

          public RobotContainer() {
            SWERVE = SwerveDrive.getInstance();
            DRIVETRAIN = DriveTrain.getInstance();
            LEDS = LEDs.getInstance();
            ELEVATOR = Elevator.getInstance();
            VISION = Vision.getInstance();
            INTAKE = new Intake();

            SWERVE.setDefaultCommand(SWERVE.joystickDrive(xBox::getLeftX, xBox::getLeftY, xBox::getRightX, SWERVE));

            configureBindings();

            loadPaths();
            autoChooser = AutoBuilder.buildAutoChooser();
            autoBranchChooser = AutoBuilder.buildAutoChooser();
          SmartDashboard.putData("Autonomous", autoChooser);
          SmartDashboard.putData("Autonomous branch", autoBranchChooser);
      }

      public void autoChangeRequest(String auto1, String auto2) {
          SmartDashboard.putData(LEDS);
      }

      private void configureBindings() {
        //xBox.b().toggleOnTrue(SWERVE.toggleSlowMode());

        xBox.leftBumper().onTrue(Commands.runOnce(() -> {
          double[] relativePosition = LimeLight.getRelativePos();
          SmartDashboard.putNumber("Position tx", relativePosition[0]);
          SmartDashboard.putNumber("Position ty", relativePosition[1]);
          SmartDashboard.putNumber("Position ta", relativePosition[2]);
          SmartDashboard.putNumber("AprilTagID", relativePosition[3]);
          LEDS.setColour(((int)relativePosition[3] % 2 == 0) ? Constants.ColourConstants.FLASHBANG : Constants.ColourConstants.PINK);
        }));

        // xBox.a().onTrue(PNEUMATICS.leftIntake());
        // xBox.x().onTrue(PNEUMATICS.rightIntake());
        // xBox.y().onTrue(PNEUMATICS.Elevator1());
        // xBox.rightBumper().onTrue(PNEUMATICS.Elevator2());
        // xBox.leftBumper().onTrue(PNEUMATICS.Elevator3());
        // xBox.x().onTrue(PNEUMATICS.toggleClimber());

        // xBox.a().onTrue(ELEVATOR.elevate(frc.robot.subsystems.Elevator.ElevatorState.DOWN));
        // xBox.b().onTrue(ELEVATOR.elevate(frc.robot.subsystems.Elevator.ElevatorState.FLOOR1));
        // xBox.x().onTrue(ELEVATOR.elevate(frc.robot.subsystems.Elevator.ElevatorState.FLOOR2));
        // xBox.y().onTrue(ELEVATOR.elevate(frc.robot.subsystems.Elevator.ElevatorState.FLOOR3));

        xBox.a().onTrue(VISION.april());
        xBox.b().onTrue(VISION.object());


      }

      private void loadPaths() {
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }

        AutoBuilder.configure(
          SWERVE::getOdometryPose,
          SWERVE::resetOdometry,
          SWERVE::getActualSpeeds,
          (speeds, feedforwards) -> SWERVE.setAutoChassisSpeeds(speeds),
          AutoConstants.autoConfig,
          config,
          () -> {
            /*if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
              return true;
            } else {
              return false;
            }*/
            return false;
          },
          SWERVE);
      }


      public Command getAutonomousCommand() {
        return autoChooser.getSelected();
      }

      public void periodic() {
        VISION.report();
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
