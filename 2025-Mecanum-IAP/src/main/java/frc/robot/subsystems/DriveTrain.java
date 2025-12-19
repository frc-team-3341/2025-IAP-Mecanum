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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
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

    private final SparkMax frontLeftMotor = new SparkMax(5,SparkMax.MotorType.kBrushless);
    private final SparkMax backLeftMotor = new SparkMax(7,SparkMax.MotorType.kBrushless);
    private final SparkMax frontRightMotor = new SparkMax(4,SparkMax.MotorType.kBrushless);
    private final SparkMax backRightMotor = new SparkMax(6,SparkMax.MotorType.kBrushless);
    private final Field2d field = new Field2d();

    private final double[] simWheelPosMeters = {0, 0, 0, 0}; //fl, fr, bl,br
    private final double[] simWheelVelMps = {0, 0, 0, 0}; //fl, fr, bl,br
    private Rotation2d simHeading = new Rotation2d();
    private double lastSimTimeSec = Timer.getFPGATimestamp();


    private MecanumDriveWheelSpeeds lastCmdWheelSpeeds = new MecanumDriveWheelSpeeds();


    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);
    //Temporary Values for the dimensions
    public static final double ROBOT_WIDTH = 0.3302; //12.4 inches in terms of width
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
        SmartDashboard.putData("Field",field);
    }
    

    private void configureMotor(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        config.encoder.positionConversionFactor(ENCODER_POSITION_CONVERSION);
        config.encoder.velocityConversionFactor(ENCODER_VELOCITY_CONVERSION);
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    }

    private MecanumDriveWheelPositions getCurrentWheelPositions() {
        if (RobotBase.isSimulation()) {
            return new MecanumDriveWheelPositions(
                simWheelPosMeters[0], simWheelPosMeters[1], simWheelPosMeters[2], simWheelPosMeters[3]
            );
        }
        return new MecanumDriveWheelPositions(
            frontLeftMotor.getEncoder().getPosition(),
            frontRightMotor.getEncoder().getPosition(),
            backLeftMotor.getEncoder().getPosition(),
            backRightMotor.getEncoder().getPosition()
        );
    }

    /** Current wheel speeds in m/s for all four wheels. 
     * There is also the simulation for the wheel speeds going on robot simulation
    */
    private MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        if (RobotBase.isSimulation()) {
            return new MecanumDriveWheelSpeeds(
                simWheelVelMps[0], simWheelVelMps[1], simWheelVelMps[2], simWheelVelMps[3]
            );
        }
        return new MecanumDriveWheelSpeeds(
            frontLeftMotor.getEncoder().getVelocity(),
            frontRightMotor.getEncoder().getVelocity(),
            backLeftMotor.getEncoder().getVelocity(),
            backRightMotor.getEncoder().getVelocity()
        );
    }

     /** Get the current estimated pose of the robot on the field.
      * The coordinates will be in meters which is very important
      * to consider when troubleshooting
      */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** Reset odometry to a known pose (used at the start of autos). 
     * Very necessary for running it again
     * or multiple times
    */
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

    /** Command robot-relative chassis speeds (used by AutoBuilder). 
     * In simpler terms, this drives the robot based off of the computed speeds 
     * from MecanumDrive and ChassisSpeed Libraries
    */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(speeds);
    }

    public void drive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(MAX_WHEEL_SPEED);
        lastCmdWheelSpeeds = wheelSpeeds;
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
        if (RobotBase.isSimulation()) {
            return simHeading;
        }
        return Rotation2d.fromDegrees(-navx.getAngle());
    }

    public void resetNavx() {
        if (RobotBase.isSimulation()) {
            simHeading = new Rotation2d();
            return;
        }
        navx.reset();
    }

    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTimeSec;
        lastSimTimeSec = now;

        if (dt <= 0.0 || dt > 0.1){
            dt = 0.02;
        }
        simWheelVelMps[0] = MathUtil.clamp(lastCmdWheelSpeeds.frontLeftMetersPerSecond, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        simWheelVelMps[1] = MathUtil.clamp(lastCmdWheelSpeeds.frontRightMetersPerSecond, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        simWheelVelMps[2] = MathUtil.clamp(lastCmdWheelSpeeds.rearLeftMetersPerSecond, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
        simWheelVelMps[3] = MathUtil.clamp(lastCmdWheelSpeeds.rearRightMetersPerSecond, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

        // wheel position
        for (int i = 0; i < 4; i++) {
            simWheelPosMeters[i] += simWheelVelMps[i] * dt;
        }

        // Estimation using chassisspeeds
        ChassisSpeeds chassis = kinematics.toChassisSpeeds(
            new MecanumDriveWheelSpeeds(simWheelVelMps[0], simWheelVelMps[1], simWheelVelMps[2], simWheelVelMps[3])
        );
        simHeading = simHeading.plus(new Rotation2d(chassis.omegaRadiansPerSecond * dt));

        // These update the sensors on simulation
        odometry.update(simHeading, getCurrentWheelPositions());

        //Pose for 2d field on advantagescope
        field.setRobotPose(getPose());
    }


    @Override
    public void periodic() {
        odometry.update(getHeadingRotation2d(), getCurrentWheelPositions());
        Pose2d pose = getPose();
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Pose X (m)", pose.getX());
        SmartDashboard.putNumber("Pose Y (m)", pose.getY());
        SmartDashboard.putNumber("Pose Theta (deg)", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("NavX Angle", navx.getAngle());
    }
}
