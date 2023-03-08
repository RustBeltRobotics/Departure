package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.*;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final CANCoder absoluteSteerEncoder;

    private final PIDController steerPID = new PIDController(STEER_P, STEER_I, STEER_D);

    public SwerveModule(int driveID, int steerID, int encoderID, double offset) {
        // Setup drive motor SparkMax
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setInverted(false);
        driveMotor.setSmartCurrentLimit(DRIVE_SMART_CURRENT_LIMIT);
        driveMotor.setSecondaryCurrentLimit(DRIVE_SECONDARY_CURRENT_LIMIT);

        // Setup drive motor relative encoder
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);
        
        // Setup steer motor SparkMax
        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setIdleMode(IdleMode.kCoast);
        steerMotor.setInverted(true);
        steerMotor.setSmartCurrentLimit(STEER_SMART_CURRENT_LIMIT);
        steerMotor.setSecondaryCurrentLimit(STEER_SECONDARY_CURRENT_LIMIT);

        // Setup steer motor relative encoder
        steerEncoder = steerMotor.getEncoder();
        steerEncoder.setPositionConversionFactor(STEER_POSITION_CONVERSION);
        steerEncoder.setVelocityConversionFactor(STEER_VELOCITY_CONVERSION);

        // Setup steer motor relative encoder
        absoluteSteerEncoder = new CANCoder(encoderID);
        absoluteSteerEncoder.configMagnetOffset(offset);

        // Allow PID to Loop over
        steerPID.enableContinuousInput(0., 360.);

        resetEncoders();
    }

    /** @return Drive position, meters, -inf to +inf */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /** @return Steer position, degrees, -inf to +inf */
    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    /** @return Drive position, meters/second */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /** @return Steer position, degrees/second */
    public double getSteerVelocity() {
        return steerEncoder.getVelocity();
    }

    /** @return Absolute steer position, degrees, 0 to 360 */
    public double getAbsolutePosition() {
        return absoluteSteerEncoder.getAbsolutePosition();
    }
    // FIXME: Confirm this is correct
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerPosition()));
      }

    /** Resets the drive relative encoder to 0 and steer relative encoder to match absolute encoder */
    public void resetEncoders() {
        driveEncoder.setPosition(0.);
        steerEncoder.setPosition(getAbsolutePosition());
    }

    /** @return The module state (velocity, m/s, and steer angle, degrees as a Rotation2d) */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /**
     * Set's the speed and angle of an idividual module.
     * @param state the desired state (velocity, m/s, and steer angle, degrees as a Rotation2d)
     */
    public void setState(SwerveModuleState state) {        
        // If input is minimal, ignore input to avoid reseting steer angle to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stopModule();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND); // TODO: Replace with tuned PID/FF
        steerMotor.set(steerPID.calculate(getSteerPosition(), state.angle.getDegrees()));
    }

    /** Set's the voltage to both motors to 0 */
    public void stopModule() {
        driveMotor.set(0.);
        steerMotor.set(0.);
    }

    /**
     * Sets the swerve module's motors to brake or coast mode.
     * @param enable true -> brake, false -> coast
     */
    public void enableBrakeMode(boolean enable) {
        if (enable) {
            driveMotor.setIdleMode(IdleMode.kBrake);
            steerMotor.setIdleMode(IdleMode.kBrake);
        } else {
            driveMotor.setIdleMode(IdleMode.kCoast);
            steerMotor.setIdleMode(IdleMode.kCoast);
        }
    }    
}
