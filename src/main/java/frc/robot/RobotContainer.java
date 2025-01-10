// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {

  private final CommandXboxController xBox = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;

  private PowerDistribution pdp;
  private SwerveDrive SWERVE;
  private LEDs LEDController;
  RobotConfig config;
  
  public RobotContainer() {
    SWERVE = new SwerveDrive();
    pdp = new PowerDistribution(0, ModuleType.kCTRE);
    LEDController = new LEDs();

    loadPaths();

    Shuffleboard.getTab("pdp").add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistribution);
    SWERVE.setDefaultCommand(SWERVE.joystickDrive(xBox::getLeftX, xBox::getLeftY, xBox::getRightX, SWERVE));
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", autoChooser);
  }

  private void configureBindings() {
    xBox.b().toggleOnTrue(SWERVE.toggleSlowMode());
    xBox.leftBumper().onTrue(Commands.runOnce(() -> {
      double[] relativePosition = LimeLight.getRelativePos();
      SmartDashboard.putNumber("Position tx", relativePosition[0]);
      SmartDashboard.putNumber("Position ty", relativePosition[1]);
      SmartDashboard.putNumber("Position ta", relativePosition[2]);
      SmartDashboard.putNumber("AprilTagID", relativePosition[3]);
      LEDController.setColour(((int)relativePosition[3] % 2 == 0) ? Constants.ColourConstants.FLASHBANG : Constants.ColourConstants.PINK);
    }));
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
      SWERVE::getChassisSpeeds,
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
}