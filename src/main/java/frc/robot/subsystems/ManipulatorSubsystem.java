package frc.robot.subsystems;

import javax.management.relation.RoleResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ManipulatorSubsystem extends SubsystemBase {

    private final IntakeModule m_IntakeModule = new IntakeModule();
    private final ShooterModule m_ShooterModule = new ShooterModule();

    private double rollerSpeed = 0;
    private int rollerDirection = 0;

    public ManipulatorSubsystem() {
        SmartDashboard.putNumber("Roller Speed", 0);
    }

    @Override
    public void periodic() {
        /**TODO
         * Any automtic behavior we want
         * An example would be if we have a note and are in the top position
         * we could start the shooting motors.
         */
        m_IntakeModule.updateIntake();
        m_ShooterModule.runShooter();
    }
    /*public void shootButtonHandler() {
        if(m_IntakeModule.getNoteLimitSwitch()){
            if(m_IntakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer){//This means you are in loading position
                m_ShooterModule.setShooterSpeed(ShooterConstants.kShootingSpeakerSpeed); 
                m_IntakeModule.setIntakeSpeed(IntakeConstants.kFeedingSpeed); 
            }
            else if (m_IntakeModule.getCurrentPosition() > IntakeConstants.kAmpShootingPosition-IntakeConstants.kArmAngleBuffer && m_IntakeModule.getCurrentPosition()< IntakeConstants.kAmpShootingPosition+IntakeConstants.kArmAngleBuffer){//This means you are aiming for the amp
                m_IntakeModule.setIntakeSpeed(IntakeConstants.kAmpShooterSpeed); 
            }
        }
    }

    public void loadPositionButtonHandler() {
        m_IntakeModule.setTargetPosition(IntakeConstants.kTopPosition);

    }

    public void ampShootPositionButtonHandler() {
        m_IntakeModule.setTargetPosition(IntakeConstants.kAmpShootingPosition);

    }

    public void intakePositionButtonHandler() {
        m_IntakeModule.setTargetPosition(IntakeConstants.kFloorIntakePosition);
    }

    public void intakeButtonHandler() {
        if(m_IntakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer){//This means you are in loading position
            m_IntakeModule.setIntakeSpeed(IntakeConstants.kIntakePlayerSpeed); 
            m_ShooterModule.setShooterSpeed(ShooterConstants.kIntakeSpeed); 
        }
        else if (m_IntakeModule.getCurrentPosition() > IntakeConstants.kFloorIntakePosition-IntakeConstants.kArmAngleBuffer && m_IntakeModule.getCurrentPosition()< IntakeConstants.kFloorIntakePosition+IntakeConstants.kArmAngleBuffer){//This means you are in shooting position
            m_IntakeModule.setIntakeSpeed(IntakeConstants.kIntakeGroundSpeed); 
            
        }
    }*/
    public void intakeButtonHandler(){
        rollerDirection = 1;
    }

    public void outputButtonHandler(){
        rollerDirection = -1;
    }

    public void stopButtonHandler(){
        rollerDirection = 0;
    }

    public void run(double armJoystick, double shootMotorSpeed){
        m_IntakeModule.setArmSpeed(armJoystick);
        m_ShooterModule.setShooterSpeed(shootMotorSpeed);
        rollerSpeed = SmartDashboard.getNumber("Roller Speed", 0);
        if (rollerSpeed > 1){
            rollerSpeed = 1;
        }
        else if (rollerSpeed < 0){
            rollerSpeed = 0;
        }
        SmartDashboard.putNumber("Roller Speed", rollerSpeed);
        m_IntakeModule.setIntakeSpeed(rollerSpeed * rollerDirection);
    }
}