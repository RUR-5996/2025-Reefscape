package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.ElevatorState;

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
          public Manipulator MANIPULATOR;
          public Intake LEFT_INTAKE;
          public Intake RIGHT_INTAKE;
          public SwerveDrive SWERVE;
          public DriveTrain DRIVETRAIN;
          private LEDs LEDS;
          RobotConfig config;
          public Pneumatics PNEUMATICS;
          public Vision VISION;
          public Climber CLIMBER;


    public RobotContainer() {
            SWERVE = SwerveDrive.getInstance();
            DRIVETRAIN = DriveTrain.getInstance();
            LEDS = LEDs.getInstance();
            ELEVATOR = Elevator.getInstance();
            MANIPULATOR = Manipulator.getInstance();
            VISION = Vision.getInstance();
            LEFT_INTAKE = new Intake(50, 51);
            RIGHT_INTAKE = new Intake(52, 53);
            CLIMBER = Climber.getInstance();

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

        // xBox.a().onTrue(VISION.april());
        // xBox.b().onTrue(VISION.object());

          xBox.a().onTrue(MANIPULATOR.dropCoralAndReturn());
          xBox.b().onTrue(ELEVATOR.checkElevator(ELEVATOR.manual, LEFT_INTAKE, RIGHT_INTAKE)); //raises to manually set height
          xBox.x().onTrue(VISION.seeAprilAndGo().andThen(ELEVATOR.checkElevator(ELEVATOR.manual, LEFT_INTAKE, RIGHT_INTAKE)).andThen(MANIPULATOR.dropCoralAndReturn()));
          xBox.y().onTrue(Commands.either(CLIMBER.climb(), CLIMBER.out(), () -> (CLIMBER.state == Climber.ClimberState.OUT)));

          xBox.leftTrigger().onTrue(LEFT_INTAKE.grabCoralSequence());
          xBox.rightTrigger().onTrue(RIGHT_INTAKE.grabCoralSequence());
          xBox.leftTrigger().onFalse(LEFT_INTAKE.releaseCoralSequence());
          xBox.rightTrigger().onFalse(RIGHT_INTAKE.releaseCoralSequence());

          xBox.povUp().onTrue(LEFT_INTAKE.intakeIn().alongWith(RIGHT_INTAKE.intakeIn()));
          xBox.povLeft().onTrue(VISION.reefMove("left"));
          xBox.povRight().onTrue(VISION.reefMove("right"));

          xBox.back().onTrue(SWERVE.resetAtReef(VISION)); // tbh idk which button this is
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
        ELEVATOR.AlgaePrioUpdate();
        ELEVATOR.checkManual();
        ELEVATOR.report();
        VISION.object(true);
        LEFT_INTAKE.prioState();
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
