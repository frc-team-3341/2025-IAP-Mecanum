// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.MecanumDrive;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final DriveTrain driveTrain = new DriveTrain();
    private final XboxController controller = new XboxController(0);
  public RobotContainer() {
    driveTrain.setDefaultCommand(
            new MecanumDrive(driveTrain, controller)
        );
  }

  public DriveTrain getDriveTrain() {
    return driveTrain;
  }

  public XboxController getController() {
    return controller;
  }
}
