// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Report;

public class Robot extends TimedRobot{
  private Command m_autonomousCommand;

  RobotContainer m_robotContainer;
  Report REPORT;
  DriveTrain DRIVETRAIN;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    REPORT = Report.getInstance();
    DRIVETRAIN = DriveTrain.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    REPORT.periodic();
  }

  @Override
  public void disabledInit() {
    DRIVETRAIN.setSteerToCoast();

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
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    DRIVETRAIN.flModule.testModule();
    DRIVETRAIN.frModule.testModule();
    DRIVETRAIN.rlModule.testModule();
    DRIVETRAIN.rrModule.testModule();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
