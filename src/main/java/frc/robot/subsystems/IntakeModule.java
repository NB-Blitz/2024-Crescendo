package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeModule {
    private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCANID, MotorType.kBrushless);
    private final CANSparkMax m_deployMotor = new CANSparkMax(IntakeConstants.kDeployMotorCANID, MotorType.kBrushless);
    private final DigitalInput m_intakeUpSwitch = new DigitalInput(IntakeConstants.kIntakeUpSwitchID);
    private final RelativeEncoder m_deployEncoder = m_deployMotor.getEncoder();
    private final DigitalInput m_noteSwitch = new DigitalInput(IntakeConstants.kNoteSwitchID);
    private final SparkPIDController m_intakePIDController = m_deployMotor.getPIDController();

    private double targetAngle = 0;
    private double rollerSpeed = 0;

    public IntakeModule() {
        m_intakePIDController.setFeedbackDevice(m_deployEncoder);
    }

    /**
     * Setting the encoder position when the intake oart is up.
     * @return True when calibration is complete, false when calibration is in progress.
     */
    public boolean calibrate() {
        if(m_intakeUpSwitch.get() == true){
            m_deployMotor.set(0);
            m_deployEncoder.setPosition(0);   
            return true;         
        }
        else{
            m_deployMotor.set(0.1);
            return false;
        }
    }

    /**
     * This functions set the target position for the PID
     * to move the intake arm to.
     * @param angle This is the target angle in degrees
     */
    public void setTargetPosition(double angle) {
        targetAngle = angle;
    }

    /**
     * This function returns the current angle of the
     * intake arm based on the encoder value.
     * @return The angle of the arm in degrees
     */
    public double getCurrentPosition() {
        return m_deployEncoder.getPosition();
    }

    /**
     * This function sets the speed of the intake roller.
     * @param speed Speed of the intake roller
     */
    public void setIntakeSpeed(double speed) {
        rollerSpeed = speed;
    }

    /**
     * This function tells whether or not we have a Note in
     * the intake.
     * @return True if we have a Note, else False
     */
    public boolean getNoteLimitSwitch() {
        return m_noteSwitch.get();
    }

    /**
     * This function should run in the periodic loop to 
     * update the motor speeds of the intake.
     */
    public void updateIntake() {
        m_intakePIDController.setReference(Math.toRadians(targetAngle), CANSparkMax.ControlType.kPosition);
        m_intakeMotor.set(rollerSpeed);
    }
}
