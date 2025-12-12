// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
    public static final double ROBOT_WIDTH  = 0.3302; //12.4 inches in terms of width
    public static final double ROBOT_LENGTH = 0.31496;//This currently 0.43 meters but may be inaccurate due to wheel positioning being possibly different
    public static final double MAX_WHEEL_SPEED = 6;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI;  //might be too slow
    private static final double ENCODER_POSITION_CONVERSION = 0.1463; // meters per rotation
    private static final double ENCODER_VELOCITY_CONVERSION = 0.002439; //3:1 gear ratio 5.5 inches wheel diameter
    private final MecanumDriveOdometry odometry;
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

        odometry = new MecanumDriveOdometry(
            kinematics,
            getHeadingRotation2d(),
            getCurrentWheelPositions()
    );

    try {
        RobotConfig config = RobotConfig.fromGUISettings(); // uses robot config from PathPlanner GUI
        AutoBuilder.configure(
            this::getPose,this::resetPose,this::getRobotRelativeSpeeds,  (speeds, feedforward) -> driveRobotRelative(speeds),new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),config,() -> {var alliance = DriverStation.getAlliance();return alliance.isPresent()
            && alliance.get() == DriverStation.Alliance.Red;},
            this
            );
        } catch (Exception e) {
            DriverStation.reportError("fail to configure autobuild", e.getStackTrace());
        }
    }

    private void configureMotor(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        config.encoder.positionConversionFactor(ENCODER_POSITION_CONVERSION);
        config.encoder.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    }

    private MecanumDriveWheelPositions getCurrentWheelPositions() {
        return new MecanumDriveWheelPositions(
                frontLeftMotor.getEncoder().getPosition(),
                frontRightMotor.getEncoder().getPosition(),
                backLeftMotor.getEncoder().getPosition(),
                backRightMotor.getEncoder().getPosition()
        );
    }

    /** Current wheel speeds in m/s for all four wheels. */
    private MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                frontLeftMotor.getEncoder().getVelocity(),
                frontRightMotor.getEncoder().getVelocity(),
                backLeftMotor.getEncoder().getVelocity(),
                backRightMotor.getEncoder().getVelocity()
        );
    }

     /** Get the current estimated pose of the robot on the field. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** Reset odometry to a known pose (used at the start of autos). */
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(
                getHeadingRotation2d(),
                getCurrentWheelPositions(),
                pose
        );
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getCurrentWheelSpeeds());
    }

    /** Command robot-relative chassis speeds (used by AutoBuilder). */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(speeds);
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
        return Rotation2d.fromDegrees(-navx.getAngle());
    }

    public void resetNavx() {
        navx.reset();
    }

    @Override
    public void periodic() {
        odometry.update(getHeadingRotation2d(), getCurrentWheelPositions());

        Pose2d pose = getPose();
        SmartDashboard.putNumber("Pose X (m)", pose.getX());
        SmartDashboard.putNumber("Pose Y (m)", pose.getY());
        SmartDashboard.putNumber("Pose Theta (deg)", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("NavX Angle", navx.getAngle());
    }
}
