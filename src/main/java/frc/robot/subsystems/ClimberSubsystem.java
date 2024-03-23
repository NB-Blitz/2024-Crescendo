package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax m_climberArmRight = new CANSparkMax(ClimberConstants.kRightMotorCANID, MotorType.kBrushless);
    private final CANSparkMax m_climberArmLeft = new CANSparkMax(ClimberConstants.kLeftMotorCANID, MotorType.kBrushless);/*TODO: Find out if the motor type is right*/
    //private boolean direction = false;

    private final DigitalInput m_leftClimberUpSwitch = new DigitalInput(ClimberConstants.kLeftClimberUpSwitchID);
    private final DigitalInput m_rightClimberUpSwitch = new DigitalInput(ClimberConstants.kRightClimberUpSwitchID);
    private final RelativeEncoder m_climberLeftArmEncoder = m_climberArmLeft.getEncoder();
    private final RelativeEncoder m_climberRightArmEncoder = m_climberArmRight.getEncoder();
    public ClimberSubsystem(){
        m_climberArmLeft.restoreFactoryDefaults();
        m_climberArmRight.restoreFactoryDefaults();


        m_climberArmLeft.setIdleMode(ClimberConstants.kArmMotorIdleMode);
        m_climberArmLeft.setSmartCurrentLimit(ClimberConstants.kArmMotorCurrentLimit);
        m_climberArmRight.setIdleMode(ClimberConstants.kArmMotorIdleMode);
        m_climberArmRight.setSmartCurrentLimit(ClimberConstants.kArmMotorCurrentLimit);

        m_climberLeftArmEncoder.setPositionConversionFactor(ClimberConstants.kClimbGearRatio);//ClimberConstants.kClimbScaleFactor);
        m_climberRightArmEncoder.setPositionConversionFactor(ClimberConstants.kClimbGearRatio);//ClimberConstants.kClimbScaleFactor);

        // Save the SPARK configurations. If a SPARK browns out during
        // operation, it will maintain the above configurations.
        m_climberArmRight.burnFlash();
        m_climberArmLeft.burnFlash();

        // Give the SPARKS time to burn the configurations to their flash.
        Timer.delay(1);
    }
    
    /*private boolean isDown(){
       if(m_climberArmEncoder.getPosition() <= ClimberConstants.kDownPosition){
            return true;
       } else{
            System.out.println("Climber arm about to self destruct ;D");
            return false;
       }
    }

    private boolean isUp(){
       if(m_climbArmEncoder.getPosition() >= ClimberConstants.kUpPosition){
            return true;
       } else{
            System.out.println("Climber arm about to self destruct ;D");
            return false;
       }
    }*/

    /*private void buttonPressed() {
          direction = !direction;
    }*/

    @Override
    public void periodic() {

    }

    public void move(double joystick) {
          /*if (direction == true) {
               if (isUp()) {
                    m_climberArmLeft.set(0);
                    m_climberArmRight.set(0);
               }
               else {
                    m_climberArmLeft.set(ClimberConstants.kClimbArmSpeed);
                    m_climberArmRight.set(ClimberConstants.kClimbArmSpeed);
               }
          }
          else {
               if (isDown()) {
                    m_climberArmLeft.set(0);
                    m_climberArmRight.set(0);
               }
               else {
                    m_climberArmLeft.set(-ClimberConstants.kClimbArmSpeed);
                    m_climberArmRight.set(-ClimberConstants.kClimbArmSpeed);
               }
          }*/
          double leftMotorSpeed = joystick;
          double rightMotorSpeed = joystick;
          if(m_leftClimberUpSwitch.get() == true && joystick<0){
               leftMotorSpeed = 0;

          }
          if(m_rightClimberUpSwitch.get() == true && joystick<0){
               rightMotorSpeed = 0;
          }
          m_climberArmLeft.set(leftMotorSpeed);
          m_climberArmRight.set(rightMotorSpeed);
          

          SmartDashboard.putNumber("Left Climber Position", m_climberLeftArmEncoder.getPosition());
          SmartDashboard.putNumber("Right Climber Position", m_climberRightArmEncoder.getPosition());

    }
}
