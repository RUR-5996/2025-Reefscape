// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ColourConstants;
import frc.robot.subsystems.LEDs;

public class Robot extends TimedRobot{
  public static Command m_autonomousCommand;

  RobotContainer m_robotContainer;
  LEDs LEDS;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    LEDS = LEDs.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {
    LEDS.setColour(ColourConstants.DARKBLUE);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {

    LEDS.setColour(ColourConstants.PINK);
  }

  @Override
  public void autonomousPeriodic() {
    //RobotContainer.check_for_auto_change_periodic();
  }

  @Override
  public void teleopInit() {
   
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
