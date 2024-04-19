// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

//import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.MAXModuleConstants;
import frc.robot.Constants.SDSModuleConstants;
import frc.robot.subsystems.swervemodules.MAXSwerveModule;
import frc.robot.subsystems.swervemodules.SDSSwerveModule;
import frc.robot.subsystems.swervemodules.SwerveModule;
import frc.utils.LimelightHelpers;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight; 
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    private double xboxSpeed = 0.5;

    // The gyro sensor
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation;
    private double m_currentTranslationDir;
    private double m_currentTranslationMag;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    private SwerveDriveKinematics m_kinematics;

    private ChassisSpeeds m_robotRelativeSpeeds = new ChassisSpeeds();

    // Odometry classes for tracking and estimating robot pose
    public SwerveDriveOdometry m_odometry;
    //public SwerveDrivePoseEstimator m_poseEstimator;

    private boolean m_demoMode = true;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        if (DriveConstants.isMAXSwerveModules) {
            m_frontLeft = new MAXSwerveModule(
                DriveConstants.kFrontLeftDrivingCanId,
                DriveConstants.kFrontLeftTurningCanId,
                MAXModuleConstants.kFrontLeftChassisAngularOffset);

            m_frontRight = new MAXSwerveModule(
                DriveConstants.kFrontRightDrivingCanId,
                DriveConstants.kFrontRightTurningCanId,
                MAXModuleConstants.kFrontRightChassisAngularOffset);

            m_backLeft = new MAXSwerveModule(
                DriveConstants.kBackLeftDrivingCanId,
                DriveConstants.kBackLeftTurningCanId,
                MAXModuleConstants.kBackLeftChassisAngularOffset);

            m_backRight = new MAXSwerveModule(
                DriveConstants.kBackRightDrivingCanId,
                DriveConstants.kBackRightTurningCanId,
                MAXModuleConstants.kBackRightChassisAngularOffset);

            m_kinematics = MAXModuleConstants.kDriveKinematics;
        } else {
            m_frontLeft = new SDSSwerveModule(
                DriveConstants.kFrontLeftDrivingCanId,
                DriveConstants.kFrontLeftTurningCanId,
                DriveConstants.kFrontLeftTurningCANcoderId,
                DriveConstants.kFrontLeftTurningOffset);

            m_frontRight = new SDSSwerveModule(
                DriveConstants.kFrontRightDrivingCanId,
                DriveConstants.kFrontRightTurningCanId,
                DriveConstants.kFrontRightTurningCANcoderId,
                DriveConstants.kFrontRightTurningOffset);

            m_backLeft = new SDSSwerveModule(
                DriveConstants.kBackLeftDrivingCanId,
                DriveConstants.kBackLeftTurningCanId,
                DriveConstants.kBackLeftTurningCANcoderId,
                DriveConstants.kBackLeftTurningOffset);

            m_backRight = new SDSSwerveModule(
                DriveConstants.kBackRightDrivingCanId,
                DriveConstants.kBackRightTurningCanId,
                DriveConstants.kBackRightTurningCANcoderId,
                DriveConstants.kBackRightTurningOffset);

            m_kinematics = SDSModuleConstants.kDriveKinematics;
        }

        SmartDashboard.putBoolean("TURBO MODE", false);

        m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            getHeading(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });
        
        /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
        below are robot specific, and should be tuned. */
        // m_poseEstimator = new SwerveDrivePoseEstimator(
        //     m_kinematics,
        //     getHeading(),
        //     new SwerveModulePosition[] {
        //         m_frontLeft.getPosition(),
        //         m_frontRight.getPosition(),
        //         m_backLeft.getPosition(),
        //         m_backRight.getPosition()
        //     },
        //     new Pose2d(),
        //     VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        //     VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            AutoConstants.kPathFollowerConfig, // HolonomicPathFollowerConfig, this should likely live in your Constants class
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        updateOdometry();

        SmartDashboard.putNumber("position_x", getPose().getX());
        SmartDashboard.putNumber("position_y", getPose().getY());
        SmartDashboard.putNumber("position_rot", getPose().getRotation().getDegrees());

        //SmartDashboard.putNumber("vision_odometry_x", getEstimatedPose().getX());
        //SmartDashboard.putNumber("vision_odometry_y", getEstimatedPose().getY());
        //SmartDashboard.putNumber("vision_odometry_rot", getEstimatedPose().getRotation().getDegrees());

        SmartDashboard.putNumber("fl_relative", m_frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("fl_absolute", m_frontLeft.getAbsoluteEncoderPos());
        SmartDashboard.putNumber("fr_relative", m_frontRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("fr_absolute", m_frontRight.getAbsoluteEncoderPos());
        SmartDashboard.putNumber("bl_relative", m_backLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("bl_absolute", m_backLeft.getAbsoluteEncoderPos());
        SmartDashboard.putNumber("br_relative", m_backRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("br_absolute", m_backRight.getAbsoluteEncoderPos());
        m_demoMode = !SmartDashboard.getBoolean("TURBO MODE", false);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    // public Pose2d getEstimatedPose() {
    //     return m_poseEstimator.getEstimatedPosition();
    // }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            getHeading(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },
            pose);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
            getHeading(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });

        // m_poseEstimator.update(
        //     getHeading(),
        //     new SwerveModulePosition[] {
        //         m_frontLeft.getPosition(),
        //         m_frontRight.getPosition(),
        //         m_backLeft.getPosition(),
        //         m_backRight.getPosition()
        //     });

        // LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        // if (limelightMeasurement.tagCount >= 2) {
        //     m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        //     m_poseEstimator.addVisionMeasurement(
        //         limelightMeasurement.pose,
        //         limelightMeasurement.timestampSeconds);
        // }
    }

    /**
     * Returns the current non-field relative chassis speeds.
     * 
     * @return Robot relative chassis speeds
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_robotRelativeSpeeds;
    }

    /**
     * Drive the robot with non-field relative chassis speeds.
     * Used for PathPlanner/autonomous.
     * 
     * @param chassisSpeeds Robot relative chassis speeds
     */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        drive(
            chassisSpeeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed,
            1,
            false,
            false);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param speedScale    Multiplier to scale the speeds by
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, double speedScale, boolean fieldRelative, boolean rateLimit) {
        double xSpeedCommanded;
        double ySpeedCommanded;
        if(m_demoMode){
            speedScale = 0.2;
        }

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }
      
            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;
      
            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);
        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * speedScale * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * speedScale * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * speedScale * DriveConstants.kMaxAngularSpeed;

        m_robotRelativeSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeading())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        
        setModuleStates(swerveModuleStates);
    }

    public void setXboxFast(){
        xboxSpeed = 1;
    }

    public void resetXboxSpeed(){
        xboxSpeed = 0.5;
    }

    /*
     * Scales the drive inputs 
     */
    public void scaledDrive(double xSpeed, double ySpeed, double rot, double speedScale){
        // Exponential Scaling
        /*double curve = 3; // 3 is almost quadratic and 2 almost is linear
        double adjustment = 2 - curve;
        double xScaled = Math.pow(curve, Math.abs(xSpeed)) - 1 + (adjustment * xSpeed);
        double yScaled = Math.pow(curve, Math.abs(ySpeed)) - 1 + (adjustment * ySpeed);
        double rotScaled = Math.pow(curve, Math.abs(rot)) - 1 + (adjustment * rot);
        */

        // Polynomial Scaling
        double exponent;
        if (IOConstants.kJoystickDrive) {
            exponent = 4;
        } else {
            exponent = 2;
            speedScale = xboxSpeed;
        }
        double xScaled = Math.pow(Math.abs(xSpeed),exponent);
        double yScaled = Math.pow(Math.abs(ySpeed),exponent);
        double rotScaled = Math.pow(Math.abs(rot),exponent);

        if (xSpeed<0) {
            xScaled = -xScaled;
        }
        if (ySpeed<0) {
            yScaled = -yScaled;
        }
        if (rot<0) {
            rotScaled = -rotScaled;
        }

        drive(xScaled, yScaled, rotScaled, speedScale, true, false);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backLeft.resetEncoders();
        m_backRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.setAngleAdjustment(0);
        m_gyro.reset();
        resetOdometry(getPose());
    }

    public void resetGyroToBackwards() {
        m_gyro.reset();
        m_gyro.setAngleAdjustment(180);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return The robot's heading in degrees, from -inf to +inf
     */
    public Rotation2d getHeading() {
        double adjustedAngle = DriveConstants.kGyroReversed ? -m_gyro.getAngle() : m_gyro.getAngle();
        return Rotation2d.fromDegrees(adjustedAngle);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void aimSpeaker() {
        //final double rotLimelight = limelightAngleProportional();
        final double strafeLimelight = limelightStrafeProportional();
        //final double forwardLimelight = limelightRangeProportional();

        driveRobotRelative(new ChassisSpeeds(0, strafeLimelight, 0));
    }

    /**
     * Simple proportional turning control with Limelight.
     * "Proportional control" is a control algorithm in which the output is proportional to the error.
     * 
     * @return Angular velocity proportional to the "tx" value from the Limelight
     */
    public double limelightAngleProportional() {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    public double limelightStrafeProportional() {
        double kP = .02;
        double targetingStrafeSpeed = LimelightHelpers.getTX("limelight") * kP;
        targetingStrafeSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        return targetingStrafeSpeed;
    }

    /**
     * Simple proportional ranging control with Limelight's "ty" value.
     * This works best if your Limelight's mount height and target mount height are different.
     * If your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty".
     * 
     * @return Forward velocity proportional to the "ty" value from the Limelight
     */
    public double limelightRangeProportional() {    
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        return targetingForwardSpeed;
    }
}
