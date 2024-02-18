package frc.robot.subsystems;

import org.ejml.dense.row.mult.MatrixMatrixMult_CDRM;

//import javax.management.relation.RoleResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.ShooterConstants;

public class ManipulatorSubsystem extends SubsystemBase {

    private final IntakeModule m_IntakeModule = new IntakeModule();
    //private final ShooterModule m_ShooterModule = new ShooterModule();

    private double rollerSpeed = 0;
    private int rollerDirection = 0;
    private double maxCurrentReachedArm = 0.0;
    private double maxCurrentReachedRoller = 0.0;
    private int rollerDirectionBanned = 0;
    private int armDirectionBanned = 0;

    public ManipulatorSubsystem() {
        SmartDashboard.putNumber("Roller Speed", 0.0);
        SmartDashboard.putNumber("Arm Angle", 0.0);
    }

    @Override
    public void periodic() {
        /**
         * TODO
         * Any automtic behavior we want
         * An example would be if we have a note and are in the top position
         * we could start the shooting motors.
         */
        m_IntakeModule.updateIntake();
        //m_ShooterModule.runShooter();
    }

    /*public void shootButtonHandler() {
        if(m_IntakeModule.getNoteLimitSwitch()){
            if(m_IntakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer){ // This means you are in loading position
                m_ShooterModule.setShooterSpeed(ShooterConstants.kShootingSpeakerSpeed);
                m_IntakeModule.setIntakeSpeed(IntakeConstants.kFeedingSpeed);
            }
            else if (m_IntakeModule.getCurrentPosition() > IntakeConstants.kAmpShootingPosition-IntakeConstants.kArmAngleBuffer && m_IntakeModule.getCurrentPosition() < IntakeConstants.kAmpShootingPosition+IntakeConstants.kArmAngleBuffer){ // This means you are aiming for the amp
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
        if(m_IntakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer){ // This means you are in loading position
            m_IntakeModule.setIntakeSpeed(IntakeConstants.kIntakePlayerSpeed);
            m_ShooterModule.setShooterSpeed(ShooterConstants.kIntakeSpeed);
        }
        else if (m_IntakeModule.getCurrentPosition() > IntakeConstants.kFloorIntakePosition-IntakeConstants.kArmAngleBuffer && m_IntakeModule.getCurrentPosition() < IntakeConstants.kFloorIntakePosition+IntakeConstants.kArmAngleBuffer){ // This means you are in shooting position
            m_IntakeModule.setIntakeSpeed(IntakeConstants.kIntakeGroundSpeed);
        }
    }*/

    public void intakeButtonHandler(){
        rollerDirection = 1;
        rollerSpeed = 0.2;
    }

    public void outputButtonHandler(){
        rollerDirection = -1;
    }

    public void stopButtonHandler(){
        rollerDirection = 0;
        rollerSpeed = 0;
    }

    public void resetEncoder() {
        m_IntakeModule.resetEncoder();
    }

    public void run(double armJoystick, double shootMotorSpeed){
        // m_ShooterModule.setShooterSpeed(shootMotorSpeed);
        // rollerSpeed = SmartDashboard.getNumber("Roller Speed", 0.0);
        // if (rollerSpeed > 1){
        //     rollerSpeed = 1;
        // }
        // else if (rollerSpeed < 0){
        //     rollerSpeed = 0;
        // }
        if (rollerDirection < 0)  {
            rollerSpeed = shootMotorSpeed;
        }

        if (m_IntakeModule.getRollerCurrent() > maxCurrentReachedRoller) {
            maxCurrentReachedRoller = m_IntakeModule.getRollerCurrent();
        }
        if (m_IntakeModule.getArmCurrent() > maxCurrentReachedArm) {
            maxCurrentReachedArm = m_IntakeModule.getArmCurrent();
        }

        /*if (rollerDirection != 0 && rollerDirection == rollerDirectionBanned) {
            rollerDirection = 0;
        }
        else {
            rollerDirectionBanned = 0;
        }*/
        if ((armJoystick <= 0 && armDirectionBanned == -1) || (armJoystick >= 0 && armDirectionBanned == 1)) {
            armJoystick = 0;
        }
        else {
            armDirectionBanned = 0;
        }

        /*if (m_IntakeModule.getRollerCurrent() > 37) {
            rollerDirectionBanned = rollerDirection;
        }*/
        if (m_IntakeModule.getArmCurrent() > 37) {
            if (armJoystick < 0) {
                armDirectionBanned = -1;
            }
            else {
                armDirectionBanned = 1;
            }
        }
        SmartDashboard.putNumber("Max Arm Current", maxCurrentReachedArm);
        SmartDashboard.putNumber("Max Roller Current", maxCurrentReachedRoller);
        SmartDashboard.putNumber("Roller Current", m_IntakeModule.getRollerCurrent());
        SmartDashboard.putNumber("Arm Current", m_IntakeModule.getArmCurrent());
        SmartDashboard.putNumber("Roller Speed", shootMotorSpeed);
        SmartDashboard.putNumber("Arm Angle", m_IntakeModule.getCurrentPosition());
        SmartDashboard.putBoolean("Arm Switch", m_IntakeModule.getArmSwitch());
        m_IntakeModule.setArmSpeed(armJoystick);
        m_IntakeModule.setIntakeSpeed(rollerSpeed * rollerDirection);
    }
}