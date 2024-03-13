package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterModule {
    private final CANSparkMax shooterMotorRight = new CANSparkMax(ShooterConstants.kRightMotorCANID, MotorType.kBrushless);
    private final CANSparkMax shooterMotorLeft = new CANSparkMax(ShooterConstants.kLeftMotorCANID, MotorType.kBrushless);
    private final SparkPIDController leftPID = shooterMotorLeft.getPIDController();
    private final SparkPIDController rightPID = shooterMotorRight.getPIDController();
    private final RelativeEncoder leftEncoder = shooterMotorLeft.getEncoder();
    private final RelativeEncoder rightEncoder = shooterMotorRight.getEncoder();

    private double wheelSpeed;

    public ShooterModule() {

        shooterMotorLeft.restoreFactoryDefaults();
        shooterMotorRight.restoreFactoryDefaults();

        leftPID.setFeedbackDevice(leftEncoder);
        rightPID.setFeedbackDevice(rightEncoder);

        shooterMotorLeft.setInverted(ShooterConstants.kInvertMotorLeft);
        shooterMotorRight.setInverted(ShooterConstants.kInvertMotorRight);

        leftPID.setP(ShooterConstants.kShooterP);
        leftPID.setI(ShooterConstants.kShooterI);
        leftPID.setD(ShooterConstants.kShooterD);
        leftPID.setFF(ShooterConstants.kShooterFF);
        leftPID.setOutputRange(
             ShooterConstants.kShooterMinOutput,
             ShooterConstants.kShooterMaxOutput);

        rightPID.setP(ShooterConstants.kShooterP);
        rightPID.setI(ShooterConstants.kShooterI);
        rightPID.setD(ShooterConstants.kShooterD);
        rightPID.setFF(ShooterConstants.kShooterFF);
        rightPID.setOutputRange(
             ShooterConstants.kShooterMinOutput,
             ShooterConstants.kShooterMaxOutput);

        shooterMotorLeft.setIdleMode(ShooterConstants.kShootMotorIdleMode);
        shooterMotorRight.setIdleMode(ShooterConstants.kShootMotorIdleMode);
        shooterMotorLeft.setSmartCurrentLimit(ShooterConstants.kShootMotorCurrentLimit);
        shooterMotorRight.setSmartCurrentLimit(ShooterConstants.kShootMotorCurrentLimit);

        // Save the SPARK configurations. If a SPARK browns out during
        // operation, it will maintain the above configurations.
        shooterMotorRight.burnFlash();
        shooterMotorLeft.burnFlash();

        // Give the SPARKS time to burn the configurations to their flash.
        Timer.delay(1);
    }

    // Sets wheelSpeed variable, double input
    public void setShooterSpeed(double wheelSpeed) {
        this.wheelSpeed = wheelSpeed;
    }

    //returns the wheelSpeed variable, double
    public double getSpeed() {
        return wheelSpeed;
    }

    //sets shooter motors using wheelSpeed variable 
    public void runShooter() {

        leftPID.setReference(wheelSpeed * NeoMotorConstants.kFreeSpeedRpm, ControlType.kVelocity);
        rightPID.setReference(wheelSpeed * NeoMotorConstants.kFreeSpeedRpm, ControlType.kVelocity);
    }
}