package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ManipulatorSubsystem extends SubsystemBase {

    private final IntakeModule m_IntakeModule = new IntakeModule();
    private final ShooterModule m_ShooterModule = new ShooterModule();

    public ManipulatorSubsystem() {

    }

    @Override
    public void periodic() {
        m_IntakeModule.updateIntake();
        m_ShooterModule.runShooter();
    }
    public void shootButtonHandler() {

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
        if(m_IntakeModule.getCurrentPosition()==IntakeConstants.kTopPosition){//This means you are in loading position
            m_IntakeModule.setIntakeSpeed(1);
            m_ShooterModule.setShooterSpeed(1);
        } else if (m_IntakeModule.getCurrentPosition()==IntakeConstants.kFloorIntakePosition){//This means you are in shooting position
            m_IntakeModule.setIntakeSpeed(1);
        }
    }
}
