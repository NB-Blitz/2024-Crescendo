package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ManipulatorSubsystem extends SubsystemBase {

    private final IntakeModule m_intakeModule = new IntakeModule();
    private final ShooterModule m_shooterModule = new ShooterModule();

    private double rollerSpeed = 0;
    private int rollerDirection = 0;
    private int shooterDirection = 0;
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
        m_intakeModule.updateIntake();
        m_shooterModule.runShooter();
    }

    public void shootButtonHandler() {
        //(m_intakeModule.getNoteLimitSwitch()){
        if(m_intakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer){ // This means you are in loading position
            //m_shooterModule.setShooterSpeed(ShooterConstants.kShootingSpeakerSpeed);
            shooterDirection = -1;
            rollerSpeed = IntakeConstants.kFeedingSpeed;
        }
        else if (m_intakeModule.getCurrentPosition() > IntakeConstants.kAmpShootingPosition-IntakeConstants.kArmAngleBuffer && m_intakeModule.getCurrentPosition() < IntakeConstants.kAmpShootingPosition+IntakeConstants.kArmAngleBuffer){ // This means you are aiming for the amp
            rollerSpeed = IntakeConstants.kAmpShooterSpeed;
        }
    }
    //}

    public void loadPositionButtonHandler() {
        m_intakeModule.setTargetPosition(IntakeConstants.kTopPosition);
    }

    public void ampShootPositionButtonHandler() {
        m_intakeModule.setTargetPosition(IntakeConstants.kAmpShootingPosition);
    }

    public void intakePositionButtonHandler() {
        m_intakeModule.setTargetPosition(IntakeConstants.kFloorIntakePosition);
    }

    public void speedHandler() {
        if(m_intakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer){ // This means you are in loading position
            rollerSpeed = IntakeConstants.kIntakePlayerSpeed;
            //m_shooterModule.setShooterSpeed(ShooterConstants.kIntakeSpeed);
            shooterDirection = 1;
        }
        else if (m_intakeModule.getCurrentPosition() > IntakeConstants.kFloorIntakePosition-IntakeConstants.kArmAngleBuffer && m_intakeModule.getCurrentPosition() < IntakeConstants.kFloorIntakePosition+IntakeConstants.kArmAngleBuffer){ // This means you are in shooting position
            rollerSpeed = IntakeConstants.kIntakeGroundSpeed;
        }
    }

    public void intakeButtonHandler(){
        rollerDirection = 1;
        rollerSpeed = IntakeConstants.kIntakeGroundSpeed;
    }

    public void outputButtonHandler(){
        rollerDirection = -1;
        rollerSpeed = IntakeConstants.kAmpShooterSpeed;
    }

    public void stopButtonHandler(){
        rollerDirection = 0;
        rollerSpeed = 0;
    }

    public void resetEncoder() {
        m_intakeModule.resetEncoder();
    }

    public void enableBounds(){
        m_intakeModule.enableBounds();
    }

    public void disableBounds(){
        m_intakeModule.disableBounds();
    }

    public void run(double armJoystick, double shootMotorSpeed){
        m_shooterModule.setShooterSpeed(shootMotorSpeed * shooterDirection);

        if (m_intakeModule.getRollerCurrent() > maxCurrentReachedRoller) {
            maxCurrentReachedRoller = m_intakeModule.getRollerCurrent();
            SmartDashboard.putNumber("Max Roller Current", maxCurrentReachedRoller);
        }
        if (m_intakeModule.getArmCurrent() > maxCurrentReachedArm) {
            maxCurrentReachedArm = m_intakeModule.getArmCurrent();
            SmartDashboard.putNumber("Max Arm Current", maxCurrentReachedArm);
        }

        /*if (rollerDirection != 0 && rollerDirection == rollerDirectionBanned) {
            rollerDirection = 0;
        }
        else {
            rollerDirectionBanned = 0;
        }
        if ((armJoystick <= 0 && armDirectionBanned == -1) || (armJoystick >= 0 && armDirectionBanned == 1)) {
            armJoystick = 0;
        }
        else {
            armDirectionBanned = 0;
        }

        if (m_intakeModule.getRollerCurrent() > 37) {
            rollerDirectionBanned = rollerDirection;
        }

        if (m_intakeModule.getArmCurrent() > 37) {
            if (armJoystick < 0) {
                armDirectionBanned = -1;
            }
            else {
                armDirectionBanned = 1;
            }
        }*/
        SmartDashboard.putNumber("Roller Current", m_intakeModule.getRollerCurrent());
        SmartDashboard.putNumber("Arm Current", m_intakeModule.getArmCurrent());
        SmartDashboard.putNumber("Roller Speed", shootMotorSpeed);
        SmartDashboard.putNumber("Arm Angle", m_intakeModule.getCurrentPosition());
        SmartDashboard.putBoolean("Arm Switch", m_intakeModule.getArmSwitch());
        SmartDashboard.putBoolean("Override Bounds", m_intakeModule.getOverrideBounds());
        m_intakeModule.setTargetVelocity(armJoystick);
        m_intakeModule.setIntakeSpeed(rollerSpeed);// * rollerDirection);
    }
}