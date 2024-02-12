package frc.robot.subsystems.swervemodules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.SwerveModuleConstants;
import frc.utils.SwerveUtils;
import frc.robot.Constants;
import frc.robot.Constants.SDSModuleConstants;

public class SDSSwerveModule extends SwerveModule {
    private final CANcoder m_turningCANcoder;
    private final RelativeEncoder m_turningEncoder;

    private final double m_turningOffset;

    /**
     * Constructs an SDSSwerveModule and configures the driving and turning motor,
     * CANcoder, and PID controller. This configuration is specific to the SDS
     * Swerve Module built with NEOs, SPARKS, and a CTRE CANcoder.
     */
    public SDSSwerveModule(int drivingCANId, int turningCANId, int turningEncoderCANId, double turningOffset) {
        super(new CANSparkMax(drivingCANId, MotorType.kBrushless), turningCANId);

        m_turningOffset = turningOffset;
        
        // Setup encoders and PID controllers for the driving and turning SPARKS.
        m_turningCANcoder = new CANcoder(turningEncoderCANId);
        MagnetSensorConfigs canCoderConfig = new MagnetSensorConfigs();
        canCoderConfig = canCoderConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        canCoderConfig = canCoderConfig.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        m_turningCANcoder.getConfigurator().apply(canCoderConfig);
        
        m_turningEncoder = super.m_turningSpark.getEncoder();
        super.m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        super.m_drivingEncoder.setPositionConversionFactor(SDSModuleConstants.kDrivingEncoderPositionFactor);
        super.m_drivingEncoder.setVelocityConversionFactor(SDSModuleConstants.kDrivingEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.setPositionConversionFactor(SDSModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(SDSModuleConstants.kTurningEncoderVelocityFactor);

        // Disable PID wrap around for the SDS turning motor.
        super.m_turningPIDController.setPositionPIDWrappingEnabled(false);

        // Set the PID gains for the driving motor.
        super.m_drivingPIDController.setP(SDSModuleConstants.kDrivingP);
        super.m_drivingPIDController.setI(SDSModuleConstants.kDrivingI);
        super.m_drivingPIDController.setD(SDSModuleConstants.kDrivingD);
        super.m_drivingPIDController.setFF(SDSModuleConstants.kDrivingFF);
        super.m_drivingPIDController.setOutputRange(
            SwerveModuleConstants.kDrivingMinOutput,
            SwerveModuleConstants.kDrivingMaxOutput);

        // Set the PID gains for the turning motor.
        super.m_turningPIDController.setP(SDSModuleConstants.kTurningP);
        super.m_turningPIDController.setI(SDSModuleConstants.kTurningI);
        super.m_turningPIDController.setD(SDSModuleConstants.kTurningD);
        super.m_turningPIDController.setFF(SDSModuleConstants.kTurningFF);
        super.m_turningPIDController.setOutputRange(
            SwerveModuleConstants.kTurningMinOutput,
            SwerveModuleConstants.kTurningMaxOutput);

        super.m_drivingSpark.setInverted(SDSModuleConstants.kDrivingInverted);
        super.m_turningSpark.setInverted(SDSModuleConstants.kTurningInverted);

        super.m_drivingSpark.setIdleMode(SwerveModuleConstants.kDrivingMotorIdleMode);
        super.m_turningSpark.setIdleMode(SwerveModuleConstants.kTurningMotorIdleMode);
        super.m_drivingSpark.setSmartCurrentLimit(SDSModuleConstants.kDrivingMotorCurrentLimit);
        super.m_turningSpark.setSmartCurrentLimit(SDSModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK configurations. If a SPARK browns out during
        // operation, it will maintain the above configurations.
        super.m_drivingSpark.burnFlash();
        super.m_turningSpark.burnFlash();

        // Give the SPARKS time to burn the configurations to their flash.
        Timer.delay(1);

        syncTurningEncoders();

        super.m_drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    @Override
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return super.getState(m_turningEncoder.getPosition(), 0);
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    @Override
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return super.getPosition(m_turningEncoder.getPosition(), 0);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees.
        // Uses a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and REV onboard are not.
        SwerveModuleState optimizedDesiredState = SwerveUtils.optimize(desiredState, getState().angle);

        // Command driving and turning SPARKS towards their respective setpoints.
        super.m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        super.m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }
    
    /**
     * Set the NEO's built-in relative encoder's position to the CANcoder's position.
     */
    public void syncTurningEncoders() {
        double absolutePosition = m_turningCANcoder.getAbsolutePosition().getValueAsDouble() * Constants.TWO_PI; // radians
        double adjustedPosition = absolutePosition - Math.toRadians(m_turningOffset);
        if (adjustedPosition < 0) {
            adjustedPosition = Constants.TWO_PI - Math.abs(adjustedPosition);
        }
        m_turningEncoder.setPosition(adjustedPosition);
    }

    /**
     * Get the CANcoder's position in degrees with the offset applied.
     * If the result is negative, adjust it to the wrapped positive value.
     * 
     * @return CANcoder minus offset position, from 0 to 360 degrees
     */
    public double getAbsoluteEncoderPos() {
        double adjustedPosition = m_turningCANcoder.getAbsolutePosition().getValueAsDouble() * 360 - m_turningOffset;
        if (adjustedPosition < 0) {
            adjustedPosition = 360 - Math.abs(adjustedPosition);
        }
        return adjustedPosition;
    }
}
