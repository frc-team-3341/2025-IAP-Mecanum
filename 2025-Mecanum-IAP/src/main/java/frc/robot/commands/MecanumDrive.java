// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class MecanumDrive extends Command {

    private final DriveTrain driveTrain;
    private final XboxController controller;

    private static final double JOYSTICK_DEADBAND = 0.05;
    private static final double MAX_TRANSLATION_SPEED = 0.5;
    private static final double ROTATION_SCALE = 0.5;
    public MecanumDrive(DriveTrain driveTrain, XboxController controller) {
        this.driveTrain = driveTrain;
        this.controller = controller;
        addRequirements(driveTrain);
    }

    private double applyDeadbandAndSquare(double val) {
        double v = MathUtil.applyDeadband(val, JOYSTICK_DEADBAND);
        return Math.copySign(v * v, v);
    }

    @Override
    public void execute() {
        double forward = applyDeadbandAndSquare(-controller.getLeftY()) * MAX_TRANSLATION_SPEED;
        double strafe  = applyDeadbandAndSquare(-controller.getLeftX()) * MAX_TRANSLATION_SPEED;
        double rotation = applyDeadbandAndSquare(-controller.getRightX()) * ROTATION_SCALE * MAX_TRANSLATION_SPEED;

        double forwardMps = forward * DriveTrain.MAX_WHEEL_SPEED;
        double strafeMps  = strafe  * DriveTrain.MAX_WHEEL_SPEED;
        double rotRadPerSec = rotation * DriveTrain.MAX_ANGULAR_SPEED_RAD_PER_SEC;
        ChassisSpeeds fieldOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardMps,
                strafeMps,
                rotRadPerSec,
                driveTrain.getHeadingRotation2d()
        );

        driveTrain.drive(fieldOrientedSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
