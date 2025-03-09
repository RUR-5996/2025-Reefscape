// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ColourConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.util.Report;
import frc.robot.util.ScoringTracker;

public class Robot extends TimedRobot{
  public static Command m_autonomousCommand;
  ScoringTracker scoringTracker;
  RobotContainer m_robotContainer;
  Report REPORT;
  DriveTrain DRIVETRAIN;
  LEDs LEDS;

  @Override
  public void robotInit() {
    scoringTracker = new ScoringTracker();
    m_robotContainer = new RobotContainer();
    REPORT = Report.getInstance();
    DRIVETRAIN = DriveTrain.getInstance();
    LEDS = LEDs.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    REPORT.periodic();
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {
    DRIVETRAIN.setSteerToCoast();
    LEDS.setColour(ColourConstants.DARKBLUE);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } else {
      System.out.println("outonomous command null");
    }
    m_robotContainer.CLIMBER.out(Commands.runOnce(() -> {}));
    LEDS.setColour(ColourConstants.PINK);
  }

  @Override
  public void autonomousPeriodic() {
    //RobotContainer.check_for_auto_change_periodic();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.CLIMBER.out(Commands.runOnce(() -> {}));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.LEFT_INTAKE.intakeOut();
    m_robotContainer.RIGHT_INTAKE.intakeOut();
    m_robotContainer.ELEVATOR.goTo(ElevatorState.FLOOR1);
    m_robotContainer.CLIMBER.climb();
  }

  @Override
  public void testPeriodic() {
    DRIVETRAIN.flModule.testModule();
    DRIVETRAIN.frModule.testModule();
    DRIVETRAIN.rlModule.testModule();
    DRIVETRAIN.rrModule.testModule();

    m_robotContainer.xBox.x().whileTrue(m_robotContainer.LEFT_INTAKE.tune());
    m_robotContainer.xBox.x().onFalse(m_robotContainer.LEFT_INTAKE.stopTune());
    m_robotContainer.xBox.b().whileTrue(m_robotContainer.RIGHT_INTAKE.tune());
    m_robotContainer.xBox.b().onFalse(m_robotContainer.RIGHT_INTAKE.stopTune());
    m_robotContainer.xBox.y().whileTrue(m_robotContainer.ELEVATOR.tune());
    m_robotContainer.xBox.y().onFalse(m_robotContainer.ELEVATOR.stopTune());
    m_robotContainer.xBox.a().whileTrue(m_robotContainer.CLIMBER.tune());
    m_robotContainer.xBox.a().onFalse(m_robotContainer.CLIMBER.stopTune());
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
