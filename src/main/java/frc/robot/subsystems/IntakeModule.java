package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.IntakeConstants;

public class IntakeModule {
    private final CANSparkMax m_armMotor = new CANSparkMax(IntakeConstants.kArmMotorCANID, MotorType.kBrushless);
    private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushed);

    private final DigitalInput m_armUpSwitch = new DigitalInput(IntakeConstants.kArmUpSwitchID);
    private final DigitalInput m_noteSwitch = new DigitalInput(IntakeConstants.kNoteSwitchID);

    private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    private final SparkPIDController m_intakePIDController = m_armMotor.getPIDController();

    private double targetAngle = IntakeConstants.kTopPosition;
    private double targetVelocity = 0;
    private double rollerSpeed = 0;
    private boolean calibrated = false;
    private boolean overrideBounds = false;
    private boolean positionMode = false;
    private boolean armUpSwitchUpdated = false;

    public IntakeModule() {
        m_armMotor.restoreFactoryDefaults();
        m_intakeMotor.restoreFactoryDefaults();

        m_intakePIDController.setFeedbackDevice(m_armEncoder);
        
        m_armMotor.setInverted(IntakeConstants.kArmInverted);
        m_intakeMotor.setInverted(IntakeConstants.kIntakeInverted);

        m_armEncoder.setPositionConversionFactor(IntakeConstants.kInatkeEncoderPositionFactor);    
        m_armEncoder.setVelocityConversionFactor(IntakeConstants.kIntakeEncoderVelocityFactor);
        m_armEncoder.setPosition(0);

         m_intakePIDController.setPositionPIDWrappingEnabled(false);

        m_intakePIDController.setP(IntakeConstants.kIntakeP);
        m_intakePIDController.setI(IntakeConstants.kIntakeI);
        m_intakePIDController.setD(IntakeConstants.kIntakeD);
        m_intakePIDController.setFF(IntakeConstants.kIntakeFF);
        m_intakePIDController.setOutputRange(
             IntakeConstants.kArmMinOutput,
             IntakeConstants.kArmMaxOutput);

        m_armMotor.setIdleMode(IntakeConstants.kArmMotorIdleMode);
        m_intakeMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        m_armMotor.setSmartCurrentLimit(IntakeConstants.kArmMotorCurrentLimit);
        m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        m_armMotor.setClosedLoopRampRate(0.5);

        // Save the SPARK configurations. If a SPARK browns out during
        // operation, it will maintain the above configurations.
        m_intakeMotor.burnFlash();
        m_armMotor.burnFlash();

        // Give the SPARKS time to burn the configurations to their flash.
        Timer.delay(1);
    }

    public double getArmCurrent() {
        return m_armMotor.getOutputCurrent();
    }

    public double getRollerCurrent() {
        return m_intakeMotor.getOutputCurrent();
    }

    /**
     * Setting the encoder position when the intake oart is up.
     * @return True when calibration is complete, false when calibration is in progress.
     */
    /*public boolean calibrate() {
        // TODO Use calibrate function in manipulator
        if (!calibrated) {
            if(getArmSwitch() == true){
                m_armMotor.set(0);
                m_armEncoder.setPosition(0);
                calibrated = true;
            }
            else{
                m_armMotor.set(0.1); //TODO Replace with constant
                calibrated = false;
            }
        }
        return calibrated;
    }*/

    /**
     * This functions set the target position for the PID
     * to move the intake arm to.
     * @param angle This is the target angle in degrees can't be greater than IntakeConstants.bottomLimit's value and cant be smaller than 0
     */
    public void setTargetPosition(double angle) {
        if (angle <= IntakeConstants.kFloorIntakePosition && angle >= IntakeConstants.kTopPosition) {
            targetAngle = angle;
            positionMode = true;
        }
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    /**
     * This function returns the current angle of the
     * intake arm based on the encoder value.
     * @return The angle of the arm in degrees
     */
    public double getCurrentPosition() {
        return m_armEncoder.getPosition();
    }

    /**
     * This function sets the speed of the intake roller.
     * @param speed Speed of the intake roller
     */
    public void setIntakeSpeed(double speed) {
        rollerSpeed = speed;
    }

    /**
     * Tells whether or not we have a Note in the intake.
     * @return True if we have a Note, else False
     */
    public boolean getNoteLimitSwitch() {
        return !m_noteSwitch.get();
    }

    /**
     * Tells whether or not the intake arm is at the home position.
     * @return True if at home, else False
     */
    public boolean getArmSwitch() {
        return !m_armUpSwitch.get();
    }

    public boolean isIntakeArmCalibrated() {
        return calibrated;
    }

    public void resetEncoder(double angle) {
        m_armEncoder.setPosition(angle);
    }

    public void enableBounds() {
        overrideBounds = false;
    }

    public void disableBounds() {
        overrideBounds = true;
        calibrated = false;
    }

    public boolean getOverrideBounds() {
        return overrideBounds;
    }

    /**
     * This function should run in the periodic loop to 
     * update the motor speeds of the intake.
     */
    public void updateIntake() {
        if (!armUpSwitchUpdated && !getArmSwitch()) {
            armUpSwitchUpdated = true;
        }

        if (getArmSwitch()) {
            m_armEncoder.setPosition(IntakeConstants.kTopPosition);
            if (armUpSwitchUpdated) {
                targetAngle = IntakeConstants.kTopPosition;
                armUpSwitchUpdated = false;
            }
            calibrated = true;
            if (targetVelocity < 0) {
                targetVelocity = 0;
                positionMode = true;
            }
        }

        // Check to see if we are allowed to exceed our bounds
        if (!overrideBounds && calibrated) {
            if (m_armEncoder.getPosition() < IntakeConstants.kTopPosition && targetVelocity < 0) {
                targetVelocity = 0;
            }
            if (m_armEncoder.getPosition() > IntakeConstants.kFloorIntakePosition && targetVelocity > 0) {
                targetVelocity = 0;
            }

            if (targetAngle > IntakeConstants.kFloorIntakePosition) {
                targetAngle = IntakeConstants.kFloorIntakePosition;
            } else if (targetAngle < IntakeConstants.kTopPosition) {
                targetAngle = IntakeConstants.kTopPosition;
            }
        }

        if (overrideBounds && getArmSwitch() && targetVelocity < 0) {
            targetVelocity = 0;
        }
  
        if (calibrated && targetVelocity == 0) {
            if (!positionMode) {
                positionMode = true;
                targetAngle = m_armEncoder.getPosition();
            }
            m_intakePIDController.setReference(targetAngle, ControlType.kPosition);
        } else {
            positionMode = false;
            m_intakePIDController.setReference(targetVelocity, ControlType.kDutyCycle);
        }

        // Set roller speed
        if (getNoteLimitSwitch() && rollerSpeed > 0) {
            rollerSpeed = 0;
        }

        m_intakeMotor.set(rollerSpeed);
    }
}
