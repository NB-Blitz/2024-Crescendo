package frc.robot.subsystems.swervemodules;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    // TODO: Should these be accessed with super. for clarity?
    protected final CANSparkBase m_drivingSpark;
    protected final CANSparkMax m_turningSpark;

    protected final RelativeEncoder m_drivingEncoder;

    protected final SparkPIDController m_drivingPIDController;
    protected final SparkPIDController m_turningPIDController;

    protected final double m_chassisAngularOffset;
    protected SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(CANSparkBase drivingSpark, int turningCANId, double chassisAngularOffset) {
        m_drivingSpark = drivingSpark;
        m_turningSpark = new CANSparkMax(turningCANId, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS to a known state before configuring
        // them. This is useful in case a SPARK is swapped out.
        m_drivingSpark.restoreFactoryDefaults();
        m_turningSpark.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the driving and turning SPARKS.
        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_drivingPIDController = m_drivingSpark.getPIDController();
        m_turningPIDController = m_turningSpark.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);

        m_chassisAngularOffset = chassisAngularOffset;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public abstract SwerveModuleState getState();

    protected SwerveModuleState getState(double turningEncoderPos, double chassisAngularOffset) {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
            new Rotation2d(turningEncoderPos - chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public abstract SwerveModulePosition getPosition();

    protected SwerveModulePosition getPosition(double turningEncoderPos, double chassisAngularOffset) {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            m_drivingEncoder.getPosition(),
            new Rotation2d(turningEncoderPos - chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public abstract void setDesiredState(SwerveModuleState desiredState);

    protected void setDesiredState(SwerveModuleState desiredState, double turningEncoderPos, double chassisAngularOffset) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(turningEncoderPos));

        // Command driving and turning SPARKS towards their respective setpoints.
        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}
