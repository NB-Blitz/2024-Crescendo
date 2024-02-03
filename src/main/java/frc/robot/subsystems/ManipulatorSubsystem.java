package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

    private final IntakeModule m_IntakeModule = new IntakeModule();
    private final ShooterModule m_ShooterModule = new ShooterModule();

    public ManipulatorSubsystem() {

    }

    @Override
    public void periodic() {

    }
    
    public void shootButtonHandler() {

    }

    public void loadPositionButtonHandler() {

    }

    public void ampShootPositionButtonHandler() {

    }

    public void intakePositionButtonHandler() {

    }

    public void intakeButtonHandler() {
        if(m_IntakeModule.getCurrentPosition()==0){//This means you are in loading position
            m_IntakeModule.setIntakeSpeed(1);
            m_ShooterModule.setShooterSpeed(1);
        } else if (m_IntakeModule.getCurrentPosition()==120){//This means you are in shooting position
            m_IntakeModule.setIntakeSpeed(1);
        }
    }
}
