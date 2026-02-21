// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  /*
   * This function is run when the robot is first started up
   * and should be used for any initialization code.
   */
  //Overriding initialization
  @Override
  public void robotInit() {
    //Instantiate RobotContainer. This performs all button bindings
    //and puts autonomous chooser onto the dashboard.
    robotContainer = new RobotContainer();
  }
  
  /*
   * This function is called every 20 ms regardless of the mode. Use for items like diagnostics
   * you want run during disabled, autonomous, teleoperated, and test.
   * 
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  //Overriding different methods in TimedRobot
  @Override
  public void robotPeriodic() {
    /*
     * Runs scheduler. Polls buttons, adds newly-scheduled
     * commands, runs already-scheduled commands, removes 
     * finished or interrupted commands, runs subsystem periodic
     * methods. Must be called from robot's periodic block in order
     * for anything in Command-based framework to work.
     */
    CommandScheduler.getInstance().run();
  }
  //This function is called once each time the robot enters Disabled mode.
  @Override
  public void disabledInit() {}

  @Override 
  public void disabledPeriodic() {}

  //This runs the autonomous command selected by RobotContainer class
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    //Schedule the autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  //This is called periodically during autonomous
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    /*
     * This makes the autonomous stop running when
     * teleop starts. If it is desirable for autonomous
     * to continue until interrupted by another
     * command, remove this line or comment it out.
     */
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

  }
  
  //This function is called periodically during operator control.
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    //Cancels all running commands at start of test mode
    CommandScheduler.getInstance().cancelAll();
  }

  //Function called periodically during test mode.
  @Override
  public void testPeriodic() {}

}