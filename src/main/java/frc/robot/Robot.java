// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
      robotContainer = new RobotContainer();
  }
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
        autonomousCommand.cancel();
        autonomousCommand = null;
    }
  }
    @Override
    public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
        autonomousCommand.schedule();
    }
    }

    @Override
    public void autonomousPeriodic() {

    }
    @Override
    public void disabledInit() {

    }

  @Override
  public void teleopPeriodic() {
    
  }
  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();
  }
}
