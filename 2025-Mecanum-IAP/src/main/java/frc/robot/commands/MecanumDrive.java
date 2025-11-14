package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class MecanumDrive extends Command {

    private final DriveTrain driveTrain;
    private final XboxController controller;

    public MecanumDrive(DriveTrain driveTrain, XboxController controller) {
        this.driveTrain = driveTrain;
        this.controller = controller;
        addRequirements(driveTrain);
    }

    private double deadband(double val) {
        return Math.abs(val) < 0.05 ? 0 : val;
    }

    @Override
    public void execute() {
        double forward = deadband(-controller.getLeftY());
        double strafe  = deadband(controller.getLeftX());
        double rotation = deadband(controller.getRightX()) * 0.5;
        //Joystick Values
        SmartDashboard.putNumber("Joystick Forward", forward);
        SmartDashboard.putNumber("Joystick Strafe", strafe);
        SmartDashboard.putNumber("Joystick Rotation", rotation);

        double robotAngleRad = Math.toRadians(driveTrain.getAngle());

        ChassisSpeeds fieldOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forward,
                strafe,
                rotation,
                new Rotation2d(robotAngleRad)
        );

        driveTrain.drive(fieldOrientedSpeeds);
        //Robot Heading
        SmartDashboard.putNumber("Robot Heading (deg)", driveTrain.getAngle());
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
