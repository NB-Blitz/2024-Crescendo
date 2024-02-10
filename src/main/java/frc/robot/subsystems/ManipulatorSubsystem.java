package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ManipulatorSubsystem extends SubsystemBase {

    private final IntakeModule m_IntakeModule = new IntakeModule();
    private final ShooterModule m_ShooterModule = new ShooterModule();

    public ManipulatorSubsystem() {

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
    public void shootButtonHandler() {
        //TODO Morning
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
    }

    public void run(double armJoystick){
        if(armJoystick != 0){ 
            double scaledInput = IntakeConstants.kJoystickScaling*armJoystick;
            double currentPos = m_IntakeModule.getCurrentPosition();
            double targetPos = currentPos+scaledInput;
            m_IntakeModule.setTargetPosition(targetPos);
        }
    }
}