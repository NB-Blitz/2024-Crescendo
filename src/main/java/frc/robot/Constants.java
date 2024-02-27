// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double TWO_PI = 2 * Math.PI;

    public static final class IOConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kManipControllerPort = 1;
        public static final double kDriveDeadband = 0.3;
        public static final double kTwistDeadband = 0.5;
        public static final int kDriveSpeedScalerAxis = 3;
        public static final int kDriveBrakeButton = 6;
        public static final int kDriveGyroResetButton = 11;

        public static final int kControllerButtonA = 1;
        public static final int kControllerButtonB = 2;
        public static final int kControllerButtonX = 3;
        public static final int kControllerButtonY = 4;
        public static final int kControllerButtonLB = 5;
        public static final int kControllerButtonRB = 6;
        public static final int kControllerButtonTwoSquares = 7;
        public static final int kControllerButtonMenu = 8;
        public static final int kControllerButtonLStick = 9;
        public static final int kControllerButtonRStick = 10;
    }

    public static final class DriveConstants {

        public static final boolean isMAXSwerveModules = true;

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = TWO_PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // SPARK CAN IDs
        public static final int kFrontLeftDrivingCanId = 2;
        public static final int kFrontRightDrivingCanId = 4;
        public static final int kBackLeftDrivingCanId = 5;
        public static final int kBackRightDrivingCanId = 8;

        public static final int kFrontLeftTurningCanId = 1;
        public static final int kFrontRightTurningCanId = 3;
        public static final int kBackLeftTurningCanId = 6;
        public static final int kBackRightTurningCanId = 7;

        public static final int kFrontLeftTurningCANcoderId = 13;
        public static final int kFrontRightTurningCANcoderId = 14;
        public static final int kBackLeftTurningCANcoderId = 15;
        public static final int kBackRightTurningCANcoderId = 16;

        // Turning encoder offsets for SDS modules
        // We can't use REV Hardware Client's calibration tool with CANcoders
        public static final double kFrontLeftTurningOffset = 110.7;
        public static final double kFrontRightTurningOffset = 241.9;
        public static final double kBackLeftTurningOffset = 199.7;
        public static final double kBackRightTurningOffset = 25.5;

        public static final boolean kGyroReversed = true;
    }

    public static final class SwerveModuleConstants {
        // Driving motor RPS required for conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final double kOpenLoopRamp = 0.5;
        public static final double kClosedLoopRamp = 0.1;
    }

    public static final class MAXModuleConstants {
        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(24.5);
        // Distance between front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(24.5);
        // Distance from the center of the robot to the farthest module
        public static final double kDriveBaseRadius = Units.inchesToMeters(17.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (SwerveModuleConstants.kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = (kWheelCircumferenceMeters
            / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = TWO_PI; // radians
        public static final double kTurningEncoderVelocityFactor = TWO_PI / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = TWO_PI; // radians

        public static final double kDrivingP = 0.04; // TODO: Tune PID values
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;

        public static final double kTurningP = 0.5;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class SDSModuleConstants {
        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(18.5);
        // Distance between front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(19.5);
        // Distance from the center of the robot to the farthest module
        public static final double kDriveBaseRadius = 0.34;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // SDS MK4 L1 driving gear ratio is 8.14:1
        public static final double kDrivingMotorReduction = 8.14;
        public static final double kDriveWheelFreeSpeedRps = (SwerveModuleConstants.kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second

        // SDS MK4 L1 turning gear ratio is 12.8:1
        public static final double kTurningMotorReduction = 12.8;
        public static final double kTurningEncoderPositionFactor = TWO_PI / kTurningMotorReduction; // radians
        public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0; // radians per second

        public static final double kDrivingP = 0.15;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 0.16;

        public static final double kTurningP = 0.45;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;

        public static final boolean kDrivingInverted = true;
        public static final boolean kTurningInverted = false;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 50; // amps
    }

    public static final class ShooterConstants {
        public static final int kLeftMotorCANID = 11; //TODO Change these CAN IDs to actual values
        public static final int kRightMotorCANID = 12; 
        public static final double kIntakeSpeed = 0.05; //speed of the shooter when intaking frm the shooter station.
        public static final double kShootingSpeakerSpeed = -0.05;
        public static final double kAmpShooterSpeed = -0.05;
    }

    public static final class IntakeConstants {
        public static final int kArmMotorCANID = 9;
        public static final int kIntakeMotorCANID = 10;
        public static final int kArmUpSwitchID = 9; // TODO: Set these
        public static final int kNoteSwitchID = -3;

        public static final boolean kIntakeInverted = true;
        public static final boolean kArmInverted = false;
        public static final double kInatkeEncoderPositionFactor = 360 / 102.4;
        public static final double kIntakeEncoderVelocityFactor = kInatkeEncoderPositionFactor / 60;

        public static final double kIntakeP = 0.075; // TODO: Change these PID values
        public static final double kIntakeI = 0;
        public static final double kIntakeD = 0;
        public static final double kIntakeFF = 0;
        public static final double kShootingMaxOutput = 0.3;
        public static final double kShootingMinOutput = -0.3;

        public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;
        public static final int kArmMotorCurrentLimit = 39;
        public static final int kIntakeMotorCurrentLimit = 39;

        public static final int kArmAngleBuffer = 10;
        public static final double kJoystickScaling = 0.5; // When the joystick is fully pressed forward, that's the number of degrees moved every 20 ms
        
        public static final double kTopPosition = 0.1; // Degrees
        public static final double kAmpShootingPosition = 82.0; // Degrees
        public static final double kFloorIntakePosition = 197.0; // Bottom angle limit in degrees, inclusive

        public static final double kAmpShooterSpeed = -0.6;
        public static final double kIntakePlayerSpeed = 0.15; // Speed of the intake motor when picking up from the player station.
        public static final double kIntakeGroundSpeed = 0.3; // Sets the motor speed of the motor on the intake arm to pick up the note
        public static final double kUnjammingSpeed = -0.05;
        public static final double kFeedingSpeed = -0.15;
        public static final double kCalibrationSpeed = 0.05;
    }

    public static final class AutoConstants {
        public static final HolonomicPathFollowerConfig kPathFollowerConfig = DriveConstants.isMAXSwerveModules ?
            new HolonomicPathFollowerConfig(
                new PIDConstants(MAXModuleConstants.kDrivingP, MAXModuleConstants.kDrivingI, MAXModuleConstants.kDrivingD), // Translation PID constants
                new PIDConstants(MAXModuleConstants.kTurningP, MAXModuleConstants.kTurningI, MAXModuleConstants.kTurningD), // Rotation PID constants
                DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                MAXModuleConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ) :
            new HolonomicPathFollowerConfig(
                new PIDConstants(SDSModuleConstants.kDrivingP, SDSModuleConstants.kDrivingI, SDSModuleConstants.kDrivingD), // Translation PID constants
                new PIDConstants(SDSModuleConstants.kTurningP, SDSModuleConstants.kTurningI, SDSModuleConstants.kTurningD), // Rotation PID constants
                DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                SDSModuleConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            );
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}
