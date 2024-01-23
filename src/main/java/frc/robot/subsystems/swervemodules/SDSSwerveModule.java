package frc.robot.subsystems.swervemodules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.SDSModuleConstants;

public class SDSSwerveModule extends SwerveModule {
    public final CANcoder m_turningCANcoder;
    private final RelativeEncoder m_turningEncoder;

    private final double m_turningOffset;

    /**
     * Constructs an SDSSwerveModule and configures the driving and turning motor,
     * CANcoder, and PID controller. This configuration is specific to the SDS
     * Swerve Module built with NEOs, SPARKS, and a CTRE CANcoder.
     */
    public SDSSwerveModule(int drivingCANId, int turningCANId, int turningEncoderCANId, double turningOffset, double chassisAngularOffset) {
        super(new CANSparkMax(drivingCANId, MotorType.kBrushless), turningCANId, chassisAngularOffset);

        m_turningOffset = turningOffset;
        
        // Setup encoders and PID controllers for the driving and turning SPARKS.
        m_turningCANcoder = new CANcoder(turningEncoderCANId);
        m_turningCANcoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1));
        m_turningEncoder = m_turningSpark.getEncoder();
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_drivingEncoder.setPositionConversionFactor(SDSModuleConstants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(SDSModuleConstants.kDrivingEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.setPositionConversionFactor(SDSModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(SDSModuleConstants.kTurningEncoderVelocityFactor);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        //m_turningEncoder.setInverted(SDSModuleConstants.kTurningEncoderInverted);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(SDSModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(SDSModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_drivingPIDController.setP(SDSModuleConstants.kDrivingP);
        m_drivingPIDController.setI(SDSModuleConstants.kDrivingI);
        m_drivingPIDController.setD(SDSModuleConstants.kDrivingD);
        m_drivingPIDController.setFF(SDSModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(
            SwerveModuleConstants.kDrivingMinOutput,
            SwerveModuleConstants.kDrivingMaxOutput);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_turningPIDController.setP(SDSModuleConstants.kTurningP);
        m_turningPIDController.setI(SDSModuleConstants.kTurningI);
        m_turningPIDController.setD(SDSModuleConstants.kTurningD);
        m_turningPIDController.setFF(SDSModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(
            SwerveModuleConstants.kTurningMinOutput,
            SwerveModuleConstants.kTurningMaxOutput);

        m_drivingSpark.setIdleMode(SwerveModuleConstants.kDrivingMotorIdleMode);
        m_turningSpark.setIdleMode(SwerveModuleConstants.kTurningMotorIdleMode);
        m_drivingSpark.setSmartCurrentLimit(SDSModuleConstants.kDrivingMotorCurrentLimit);
        m_turningSpark.setSmartCurrentLimit(SDSModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK configurations. If a SPARK browns out during
        // operation, it will maintain the above configurations.
        m_drivingSpark.burnFlash();
        m_turningSpark.burnFlash();

        Timer.delay(1.0);
        syncTurningEncoders();

        m_desiredState.angle = new Rotation2d(Math.abs(m_turningEncoder.getPosition() % (2 * Math.PI)));
        m_drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return super.getState(Math.abs(m_turningEncoder.getPosition() % (2 * Math.PI)), m_chassisAngularOffset);
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return super.getPosition(m_turningEncoder.getPosition(), m_chassisAngularOffset);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        super.setDesiredState(desiredState, Math.abs(m_turningEncoder.getPosition() % (2 * Math.PI)), m_chassisAngularOffset);
    }
    
    public void syncTurningEncoders() {
        // TODO: Review this
        double absolutePosition = m_turningCANcoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI; // radians
        double adjustedPosition = absolutePosition - Math.toRadians(m_turningOffset);
        m_turningEncoder.setPosition(adjustedPosition);
    }

    public double getCANcoderPosition() {
        return m_turningCANcoder.getAbsolutePosition().getValueAsDouble() * 360 - m_turningOffset;
    }
}
