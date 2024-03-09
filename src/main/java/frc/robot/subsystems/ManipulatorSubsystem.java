package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ManipulatorSubsystem extends SubsystemBase {

    private final IntakeModule m_intakeModule = new IntakeModule();
    private final ShooterModule m_shooterModule = new ShooterModule();

    private double m_rollerSpeed = 0;
    private double m_shooterSpeed = 0;
    private double maxCurrentReachedArm = 0.0;
    private double maxCurrentReachedRoller = 0.0;

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

    public void shootButtonHandler(boolean runIntake) {
        //(m_intakeModule.getNoteLimitSwitch()){
        if(m_intakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer){ // This means you are in loading position
            m_shooterSpeed = ShooterConstants.kShootingSpeakerSpeed;
            if(runIntake) {
                m_rollerSpeed = IntakeConstants.kFeedingSpeed;
            }
        }
        else if (m_intakeModule.getCurrentPosition() > IntakeConstants.kAmpShootingPosition-IntakeConstants.kArmAngleBuffer && m_intakeModule.getCurrentPosition() < IntakeConstants.kAmpShootingPosition+IntakeConstants.kArmAngleBuffer){ // This means you are aiming for the amp
            m_rollerSpeed = IntakeConstants.kAmpShooterSpeed;
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

    public void intakeButtonHandler() {
        if (!m_intakeModule.getNoteLimitSwitch()){
            if(m_intakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer){ // This means you are in loading position
                m_rollerSpeed = IntakeConstants.kIntakePlayerSpeed;
                m_shooterSpeed = ShooterConstants.kIntakeSpeed;
            }
            else if (m_intakeModule.getCurrentPosition() > IntakeConstants.kFloorIntakePosition-IntakeConstants.kArmAngleBuffer && m_intakeModule.getCurrentPosition() < IntakeConstants.kFloorIntakePosition+IntakeConstants.kArmAngleBuffer){ // This means you are in shooting position
                m_rollerSpeed = IntakeConstants.kIntakeGroundSpeed;
            }
        }
    }

    public void stopButtonHandler(){
        m_rollerSpeed = 0;
        m_shooterSpeed = 0;
    }

    public void resetEncoder() {
        m_intakeModule.resetEncoder(IntakeConstants.kTopPosition);
        m_intakeModule.setTargetPosition(IntakeConstants.kTopPosition);
    }

    public void enableBounds(){
        m_intakeModule.enableBounds();
    }

    public void disableBounds(){
        m_intakeModule.disableBounds();
    }

    public void run(double armJoystick){
        m_shooterModule.setShooterSpeed(m_shooterSpeed);

        if (m_intakeModule.getRollerCurrent() > maxCurrentReachedRoller) {
            maxCurrentReachedRoller = m_intakeModule.getRollerCurrent();
            SmartDashboard.putNumber("Max Roller Current", maxCurrentReachedRoller);
        }
        if (m_intakeModule.getArmCurrent() > maxCurrentReachedArm) {
            maxCurrentReachedArm = m_intakeModule.getArmCurrent();
            SmartDashboard.putNumber("Max Arm Current", maxCurrentReachedArm);
        }

        SmartDashboard.putNumber("Roller Current", m_intakeModule.getRollerCurrent());
        SmartDashboard.putNumber("Arm Current", m_intakeModule.getArmCurrent());
        SmartDashboard.putNumber("Roller Speed", m_shooterSpeed);
        SmartDashboard.putNumber("Arm Angle", m_intakeModule.getCurrentPosition());
        SmartDashboard.putBoolean("Arm Switch", m_intakeModule.getArmSwitch());
        SmartDashboard.putBoolean("Override Bounds", m_intakeModule.getOverrideBounds());
        m_intakeModule.setTargetVelocity(armJoystick);
        m_intakeModule.setIntakeSpeed(m_rollerSpeed);
    }
}