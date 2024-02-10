package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;


public class ShooterModule {
    
    private final CANSparkMax shooterMotorRight;
    private final CANSparkMax shooterMotorLeft;

    private double wheelSpeed;

    // TODO Afternoon Configure Motor Controllers
    public ShooterModule() {
        shooterMotorLeft = new CANSparkMax(ShooterConstants.kLeftMotorCANID, MotorType.kBrushless);
        shooterMotorRight = new CANSparkMax(ShooterConstants.kRightMotorCANID, MotorType.kBrushless);
    }

    // Sets wheelSpeed variable, double input
    public void setShooterSpeed(double wheelSpeed) {
        this.wheelSpeed = wheelSpeed;
    }

    //returns the wheelSpeed variable, double
    public double getSpeed()
    {
        return wheelSpeed;
    }

    //sets shooter motors using wheelSpeed variable 
    public void runShooter() {
        shooterMotorLeft.set(wheelSpeed);
        shooterMotorRight.set(-wheelSpeed);
    }
}
