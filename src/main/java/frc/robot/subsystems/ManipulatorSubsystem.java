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
        m_IntakeModule.setTargetPosition(0);

    }

    public void ampShootPositionButtonHandler() {
        m_IntakeModule.setTargetPosition(90);

    }

    public void intakePositionButtonHandler() {

    }

    public void intakeButtonHandler() {
        
    }
}
