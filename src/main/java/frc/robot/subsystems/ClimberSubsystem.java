package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import javax.management.relation.RelationService;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax m_climberArmRight = new CANSparkMax(ClimberConstants.kRightMotorCANID, MotorType.kBrushless);
    private final CANSparkMax m_climberArmLeft = new CANSparkMax(ClimberConstants.kLeftMotorCANID, MotorType.kBrushless);/*TODO: Find out if the motor type is right*/

    private final RelativeEncoder m_climberArmEncoder = m_climberArmLeft.getEncoder();
    public ClimberSubsystem(){
        m_climberArmLeft.restoreFactoryDefaults();
        m_climberArmRight.restoreFactoryDefaults();


        m_climberArmLeft.setIdleMode(ClimberConstants.kArmMotorIdleMode);
        m_climberArmLeft.setSmartCurrentLimit(ClimberConstants.kArmMotorCurrentLimit);
        m_climberArmRight.setIdleMode(ClimberConstants.kArmMotorIdleMode);
        m_climberArmRight.setSmartCurrentLimit(ClimberConstants.kArmMotorCurrentLimit);
        
        // Save the SPARK configurations. If a SPARK browns out during
        // operation, it will maintain the above configurations.
        m_climberArmRight.burnFlash();
        m_climberArmLeft.burnFlash();

        // Give the SPARKS time to burn the configurations to their flash.
        Timer.delay(1);
    }
    
    public boolean isClimbArmDown(){
       if(m_climberArmEncoder.getPosition() == ClimberConstants.kDownPosition){
            return true;
       }else if(m_climberArmEncoder.getPosition() == ClimberConstants.kUpPosition){
            return false;
       }else{
            System.out.println("Climber arm about to self destruct ;D");
            return false;
       }
    }
    
}
