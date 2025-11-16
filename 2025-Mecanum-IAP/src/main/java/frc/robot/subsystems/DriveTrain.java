// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//Studica Imports
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class DriveTrain extends SubsystemBase {

    private final SparkMax frontLeftMotor  = new SparkMax(1,SparkMax.MotorType.kBrushless);
    private final SparkMax backLeftMotor   = new SparkMax(2,SparkMax.MotorType.kBrushless);
    private final SparkMax frontRightMotor = new SparkMax(3,SparkMax.MotorType.kBrushless);
    private final SparkMax backRightMotor  = new SparkMax(4,SparkMax.MotorType.kBrushless);

    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);
    //Temporary Values for the dimensions
    public static final double ROBOT_WIDTH  = 0.32; //Orignally, this was 12.6 inches in terms of width
    public static final double ROBOT_LENGTH = 0.43;//This currently 0.43 meters but may be inaccurate due to wheel positioning being possibly different
    public static final double MAX_WHEEL_SPEED = 4.0;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI; 
    //Left is + and forward is also +
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            new Translation2d(ROBOT_LENGTH / 2,  ROBOT_WIDTH / 2),   // Front Left
            new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),   // Front Right
            new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),   // Back Left
            new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2)   // Back Right
    );

    public DriveTrain() {
        // Configure motors with correct inversion
        configureMotor(frontLeftMotor, true);
        configureMotor(backLeftMotor,  true);
        configureMotor(frontRightMotor,false);
        configureMotor(backRightMotor, false);
    }

    private void configureMotor(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    }

    public void drive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(MAX_WHEEL_SPEED);
        frontLeftMotor.set(wheelSpeeds.frontLeftMetersPerSecond / MAX_WHEEL_SPEED);
        frontRightMotor.set(wheelSpeeds.frontRightMetersPerSecond / MAX_WHEEL_SPEED);
        backLeftMotor.set(wheelSpeeds.rearLeftMetersPerSecond / MAX_WHEEL_SPEED);
        backRightMotor.set(wheelSpeeds.rearRightMetersPerSecond / MAX_WHEEL_SPEED);
        //Displaying wheelspeed on shufflboard
        SmartDashboard.putNumber("FL Wheel Speed(m/s)", wheelSpeeds.frontLeftMetersPerSecond);
        SmartDashboard.putNumber("FR Wheel Speed(m/s)", wheelSpeeds.frontRightMetersPerSecond);
        SmartDashboard.putNumber("BL Wheel Speed(m/s)", wheelSpeeds.rearLeftMetersPerSecond);
        SmartDashboard.putNumber("BR Wheel Speed(m/s)", wheelSpeeds.rearRightMetersPerSecond);
    }

    public void drive(double forwardMps, double strafeMps, double rotRadPerSec) {
        drive(new ChassisSpeeds(forwardMps, strafeMps, rotRadPerSec));
    }

    public Rotation2d getHeadingRotation2d() {
        double wrapped = Math.IEEEremainder(navx.getAngle(), 360.0);
        //neg for field centric
        return Rotation2d.fromDegrees(-wrapped);
    }

    public void resetNavx() {
        navx.reset();
    }

    @Override
    public void periodic() {
        //Readings
        SmartDashboard.putNumber("NavX Angle", navx.getAngle());
    }
}
