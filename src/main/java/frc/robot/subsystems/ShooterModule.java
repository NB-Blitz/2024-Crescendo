package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterModule {
    private final CANSparkMax shooterMotorRight;
    private final CANSparkMax shooterMotorLeft;

    private double wheelSpeed;

    // TODO Afternoon Configure Motor Controllers
    public ShooterModule() {
        shooterMotorLeft = new CANSparkMax(ShooterConstants.kLeftMotorCANID, MotorType.kBrushless);
        shooterMotorRight = new CANSparkMax(ShooterConstants.kRightMotorCANID, MotorType.kBrushless);
        shooterMotorLeft.restoreFactoryDefaults();
        shooterMotorRight.restoreFactoryDefaults();


        shooterMotorLeft.setIdleMode(IntakeConstants.kArmMotorIdleMode);
        shooterMotorRight.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        shooterMotorLeft.setSmartCurrentLimit(IntakeConstants.kArmMotorCurrentLimit);
        shooterMotorRight.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

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
        shooterMotorLeft.set(-wheelSpeed);
        shooterMotorRight.set(wheelSpeed);
    }
}