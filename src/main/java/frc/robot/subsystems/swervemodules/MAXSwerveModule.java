package frc.robot.subsystems.swervemodules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;

import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.MAXModuleConstants;

public class MAXSwerveModule extends SwerveModule {
    private final AbsoluteEncoder m_turningEncoder;

    private final double m_chassisAngularOffset;

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        super(new CANSparkFlex(drivingCANId, MotorType.kBrushless), turningCANId);

        // Setup encoders and PID controllers for the driving and turning SPARKS.
        m_turningEncoder = super.m_turningSpark.getAbsoluteEncoder(Type.kDutyCycle);
        super.m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        super.m_drivingEncoder.setPositionConversionFactor(MAXModuleConstants.kDrivingEncoderPositionFactor);
        super.m_drivingEncoder.setVelocityConversionFactor(MAXModuleConstants.kDrivingEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.setPositionConversionFactor(MAXModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(MAXModuleConstants.kTurningEncoderVelocityFactor);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        m_turningEncoder.setInverted(MAXModuleConstants.kTurningEncoderInverted);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        super.m_turningPIDController.setPositionPIDWrappingEnabled(true);
        super.m_turningPIDController.setPositionPIDWrappingMinInput(MAXModuleConstants.kTurningEncoderPositionPIDMinInput);
        super.m_turningPIDController.setPositionPIDWrappingMaxInput(MAXModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        super.m_drivingPIDController.setP(MAXModuleConstants.kDrivingP);
        super.m_drivingPIDController.setI(MAXModuleConstants.kDrivingI);
        super.m_drivingPIDController.setD(MAXModuleConstants.kDrivingD);
        super.m_drivingPIDController.setFF(MAXModuleConstants.kDrivingFF);
        super.m_drivingPIDController.setOutputRange(
            SwerveModuleConstants.kDrivingMinOutput,
            SwerveModuleConstants.kDrivingMaxOutput);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        super.m_turningPIDController.setP(MAXModuleConstants.kTurningP);
        super.m_turningPIDController.setI(MAXModuleConstants.kTurningI);
        super.m_turningPIDController.setD(MAXModuleConstants.kTurningD);
        super.m_turningPIDController.setFF(MAXModuleConstants.kTurningFF);
        super.m_turningPIDController.setOutputRange(
            SwerveModuleConstants.kTurningMinOutput,
            SwerveModuleConstants.kTurningMaxOutput);

        super.m_drivingSpark.setIdleMode(SwerveModuleConstants.kDrivingMotorIdleMode);
        super.m_turningSpark.setIdleMode(SwerveModuleConstants.kTurningMotorIdleMode);
        super.m_drivingSpark.setSmartCurrentLimit(MAXModuleConstants.kDrivingMotorCurrentLimit);
        super.m_turningSpark.setSmartCurrentLimit(MAXModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK configurations. If a SPARK browns out during
        // operation, it will maintain the above configurations.
        super.m_drivingSpark.burnFlash();
        super.m_turningSpark.burnFlash();

        // Give the SPARKS time to burn the configurations to their flash.
        Timer.delay(1);
        
        super.m_drivingEncoder.setPosition(0);

        m_chassisAngularOffset = chassisAngularOffset;
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
        return super.getState(m_turningEncoder.getPosition(), m_chassisAngularOffset);
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
        return super.getPosition(m_turningEncoder.getPosition(), m_chassisAngularOffset);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(m_turningEncoder.getPosition()));

        // Command driving and turning SPARKS towards their respective setpoints.
        super.m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        super.m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }
}
