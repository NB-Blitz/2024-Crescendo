package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeFloor;
import frc.robot.commands.IntakeHome;
import frc.robot.commands.ShootSpeaker;

public class ManipulatorSubsystem extends SubsystemBase {
    private final IntakeModule m_intakeModule = new IntakeModule();
    private final ShooterModule m_shooterModule = new ShooterModule();

    public enum ARM_POS {
        HOME,
        AMP,
        FLOOR,
        FREE
    }
    public ARM_POS currentPos = ARM_POS.HOME;

    private double m_rollerSpeed = 0;
    private double m_shooterSpeed = 0;

    private boolean debugMode = false;

    public ManipulatorSubsystem() {}

    @Override
    public void periodic() {
        /**
         * Any automtic behavior we want
         * An example would be if we have a note and are in the top position
         * we could start the shooting motors.
         */
        m_intakeModule.updateIntake();
        m_shooterModule.runShooter();

        SmartDashboard.putNumber("Arm Angle", m_intakeModule.getCurrentPosition());
        SmartDashboard.putBoolean("Arm Switch", m_intakeModule.getArmSwitch());
        SmartDashboard.putBoolean("Note Switch", m_intakeModule.getNoteLimitSwitch());
        SmartDashboard.putBoolean("Arm Calibrated", m_intakeModule.isIntakeArmCalibrated());
        SmartDashboard.putBoolean("Debug Mode", debugMode);
        SmartDashboard.putBoolean("Override Bounds", m_intakeModule.getOverrideBounds());
    }

    public void homePositionButtonHandler() {
        m_intakeModule.setTargetPosition(IntakeConstants.kTopPosition);
    }

    public void ampShootPositionButtonHandler() {
        m_intakeModule.setTargetPosition(IntakeConstants.kAmpShootingPosition);
    }

    public void floorPositionButtonHandler() {
        m_intakeModule.setTargetPosition(IntakeConstants.kFloorIntakePosition);
    }

    public void intakeButtonHandler() {
        if (!m_intakeModule.getNoteLimitSwitch()) {
            if(m_intakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer) { // This means you are in loading position
                m_rollerSpeed = IntakeConstants.kIntakePlayerSpeed;
                m_shooterSpeed = ShooterConstants.kIntakeSpeed;
            }
            else if (m_intakeModule.getCurrentPosition() > IntakeConstants.kFloorIntakePosition-IntakeConstants.kArmAngleBuffer &&
                    m_intakeModule.getCurrentPosition() < IntakeConstants.kFloorIntakePosition+IntakeConstants.kArmAngleBuffer) { // This means you are in shooting position
                m_rollerSpeed = IntakeConstants.kIntakeGroundSpeed;
            }
        }
    }

    public void shootButtonHandler(boolean runIntake) {
        if(m_intakeModule.getCurrentPosition() < IntakeConstants.kTopPosition+IntakeConstants.kArmAngleBuffer) { // This means you are in loading position
            m_shooterSpeed = ShooterConstants.kShootingSpeakerSpeed;
            if(runIntake) {
                m_rollerSpeed = IntakeConstants.kFeedingSpeed;
            }
        }
        else if (m_intakeModule.getCurrentPosition() > IntakeConstants.kAmpShootingPosition-IntakeConstants.kArmAngleBuffer &&
                m_intakeModule.getCurrentPosition() < IntakeConstants.kAmpShootingPosition+IntakeConstants.kArmAngleBuffer) { // This means you are aiming for the amp
            m_rollerSpeed = IntakeConstants.kAmpShooterSpeed;
        }
    }

    public void stopButtonHandler() {
        m_rollerSpeed = 0;
        m_shooterSpeed = 0;
    }

    public void setDebugMode(boolean enabled) {
        debugMode = enabled;
    }

    public void resetEncoder() {
        if (debugMode) {
            m_intakeModule.resetEncoder(IntakeConstants.kTopPosition);
            m_intakeModule.setTargetPosition(IntakeConstants.kTopPosition);
        }
    }

    public void usePositionBounds(boolean enabled) {
        if (debugMode) {
            if (enabled) {
                m_intakeModule.enableBounds();
            } else {
                m_intakeModule.disableBounds();
            }
        }
    }

    public void run(double armJoystick) {
        if (m_intakeModule.getNoteLimitSwitch()) {
            if (m_rollerSpeed > 0) {
                m_rollerSpeed = 0;
            }
            if (m_shooterSpeed > 0) {
                m_shooterSpeed = 0;
            }
        }

        m_shooterModule.setShooterSpeed(m_shooterSpeed);
        m_intakeModule.setTargetVelocity(armJoystick);
        m_intakeModule.setIntakeSpeed(m_rollerSpeed);

        double currentArmPos = m_intakeModule.getCurrentPosition();
        if (currentArmPos == IntakeConstants.kTopPosition) {
            currentPos = ARM_POS.HOME;
        } else if (currentArmPos <= IntakeConstants.kAmpShootingPosition + 1 && currentArmPos >= IntakeConstants.kAmpShootingPosition - 1) {
            currentPos = ARM_POS.AMP;
        } else if (currentArmPos >= IntakeConstants.kFloorIntakePosition) {
            currentPos = ARM_POS.FLOOR;
        } else {
            currentPos = ARM_POS.FREE;
        }
    }

    public boolean getNoteSwitch() {
        return m_intakeModule.getNoteLimitSwitch();
    }

    public Command intakeRollersCommand() {
        return new Intake(this);
    }

    public Command intakeFloorPosCommand() {
        return new IntakeFloor(this);
    }

    public Command intakeHomePosCommand() {
        return new IntakeHome(this);
    }

    public Command shootSpeakerCommand() {
        return new ShootSpeaker(this);
    }
}