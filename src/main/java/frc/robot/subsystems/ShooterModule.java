package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class ShooterModule {
    
    private final TalonSRX shooterMotorRight;
    private final TalonSRX shooterMotorLeft;

    private double wheelSpeed;

    public ShooterModule() {

        this.shooterMotorLeft = //new talonSRX(constant);
        this.shooterMotorRight = //;

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
        shooterMotorLeft.speed(wheelSpeed);
        shooterMotorRight.speed(-wheelSpeed);

    }

}
