package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;

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
    private boolean calibrated = false;

    public IntakeModule() {
        m_deployMotor.restoreFactoryDefaults();
        m_intakeMotor.restoreFactoryDefaults();
        
        m_deployEncoder.setPositionConversionFactor(IntakeConstants.kInatkeEncoderPositionFactor);    
        m_deployEncoder.setVelocityConversionFactor(IntakeConstants.kIntakeEncoderVelocityFactor);
        m_intakePIDController.setFeedbackDevice(m_deployEncoder);
        m_intakePIDController.setPositionPIDWrappingEnabled(false);
        m_intakePIDController.setP(IntakeConstants.kIntakeP);
        m_intakePIDController.setI(IntakeConstants.kIntakeI);
        m_intakePIDController.setD(IntakeConstants.kIntakeD);
        m_intakePIDController.setFF(IntakeConstants.kIntakeFF);
        m_intakePIDController.setOutputRange(
            IntakeConstants.kShootingMinOutput,
            IntakeConstants.kShootingMaxOutput);
        m_intakeMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        m_deployMotor.setIdleMode(IntakeConstants.kDeployMotorIdleMode);
        m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        m_deployMotor.setSmartCurrentLimit(IntakeConstants.kDeployMotorCurrentLimit);

        // Save the SPARK configurations. If a SPARK browns out during
        // operation, it will maintain the above configurations.
        m_intakeMotor.burnFlash();
        m_deployMotor.burnFlash();

        // Give the SPARKS time to burn the configurations to their flash.
        Timer.delay(1);
    }

    /**
     * Setting the encoder position when the intake oart is up.
     * @return True when calibration is complete, false when calibration is in progress.
     */
    public boolean calibrate() {
        // TODO Afternoon use calibrate function in manipulator
        if (!calibrated) {
            if(m_intakeUpSwitch.get() == true){
                m_deployMotor.set(0);
                m_deployEncoder.setPosition(0);
                calibrated = true;
            }
            else{
                m_deployMotor.set(0.1); //TODO Afternoon replace with constant
                calibrated = false;
            }
        }
        return calibrated;
    }

    /**
     * This functions set the target position for the PID
     * to move the intake arm to.
     * @param angle This is the target angle in degrees can't be greater than IntakeConstants.bottomLimit's value and cant be smaller than 0
     */
    public void setTargetPosition(double angle) {
        // TODO Afternoon might be worth checking the top limit switch
        if( angle >= IntakeConstants.kFloorIntakePosition && angle <= IntakeConstants.kTopPosition) {
            targetAngle = angle;
        }
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
        if(m_noteSwitch.get() == true && speed > 0){
            rollerSpeed = 0;
        }else{
            rollerSpeed = speed;
        }
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
        // if we have a note and roller speed is positive, set roller speed to 0
        if(m_noteSwitch.get() && rollerSpeed > 0){
            rollerSpeed = 0;
        }
        //if the top limit switch is pressed, set the relative encoder position to 0
        if(m_intakeUpSwitch.get()){
            m_deployEncoder.setPosition(0.0);
        }
        m_intakeMotor.set(rollerSpeed);
        m_intakePIDController.setReference(Math.toRadians(targetAngle), CANSparkMax.ControlType.kPosition);
    }
}
