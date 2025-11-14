// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Studica Imports
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class DriveTrain extends SubsystemBase {

    private final PWMSparkMax frontLeftMotor  = new PWMSparkMax(0);
    private final PWMSparkMax backLeftMotor   = new PWMSparkMax(1);
    private final PWMSparkMax frontRightMotor = new PWMSparkMax(2);
    private final PWMSparkMax backRightMotor  = new PWMSparkMax(3);

    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

    private static final double ROBOT_WIDTH  = 0.6;
    private static final double ROBOT_LENGTH = 0.6;
    private static final double MAX_WHEEL_SPEED = 1.0;

    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            new Translation2d(ROBOT_LENGTH / 2,  ROBOT_WIDTH / 2),   // Front Left
            new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),   // Front Right
            new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),   // Back Left
            new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2)   // Back Right
    );

    public DriveTrain() {
        frontLeftMotor.setInverted(true);
        backLeftMotor.setInverted(true);
    }

    public void drive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(MAX_WHEEL_SPEED);

        frontLeftMotor.set(wheelSpeeds.frontLeftMetersPerSecond / MAX_WHEEL_SPEED);
        frontRightMotor.set(wheelSpeeds.frontRightMetersPerSecond / MAX_WHEEL_SPEED);
        backLeftMotor.set(wheelSpeeds.rearLeftMetersPerSecond / MAX_WHEEL_SPEED);
        backRightMotor.set(wheelSpeeds.rearRightMetersPerSecond / MAX_WHEEL_SPEED);
        //Displaying wheelspeed on shufflboard
        SmartDashboard.putNumber("FL Wheel Speed", wheelSpeeds.frontLeftMetersPerSecond);
        SmartDashboard.putNumber("FR Wheel Speed", wheelSpeeds.frontRightMetersPerSecond);
        SmartDashboard.putNumber("BL Wheel Speed", wheelSpeeds.rearLeftMetersPerSecond);
        SmartDashboard.putNumber("BR Wheel Speed", wheelSpeeds.rearRightMetersPerSecond);
    }

    public void drive(double forward, double strafe, double rotation) {
        drive(new ChassisSpeeds(forward, strafe, rotation));
    }

    public double getAngle() {
        return navx.getAngle();
    }

    public void resetNavx() {
        navx.reset();
    }

    @Override
    public void periodic() {
        //Readings
        SmartDashboard.putNumber("NavX Angle", getAngle());
        SmartDashboard.putNumber("Front Left Motor Output", frontLeftMotor.get());
        SmartDashboard.putNumber("Front Right Motor Output", frontRightMotor.get());
        SmartDashboard.putNumber("Back Left Motor Output", backLeftMotor.get());
        SmartDashboard.putNumber("Back Right Motor Output", backRightMotor.get());
    }
}
