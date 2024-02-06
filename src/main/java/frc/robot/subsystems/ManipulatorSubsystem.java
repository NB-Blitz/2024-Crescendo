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
        /**TODO
         * Any automtic behavior we want
         * An example would be if we have a note and are in the top position
         * we could start the shooting motors.
         */
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

    //TODO add small tolerances to the position requirements
    public void intakeButtonHandler() {
        if(m_IntakeModule.getCurrentPosition()==IntakeConstants.kTopPosition){//This means you are in loading position
            m_IntakeModule.setIntakeSpeed(1); //TODO replace with a constant
            m_ShooterModule.setShooterSpeed(1); //TODO replace with a constant
        }
        else if (m_IntakeModule.getCurrentPosition()==IntakeConstants.kFloorIntakePosition){//This means you are in shooting position
            m_IntakeModule.setIntakeSpeed(1); //TODO replace with a constant
        }
    }

    public void run(){//TODO Add joy stick axis as a paramter
        /*TODO
         * Check if there is joystick input
         *      if there is, set the target postition of the arm
         *      the current position + (axis value * scaling constant)
         */
    }
}
