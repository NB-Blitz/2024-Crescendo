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
        m_IntakeModule.setTargetPosition(120);
    }

    public void intakeButtonHandler() {
        
    }
}
