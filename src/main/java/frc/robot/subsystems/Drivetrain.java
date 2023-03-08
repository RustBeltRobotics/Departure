package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
    // NavX connected over MXP
    public final AHRS navx = new AHRS(SPI.Port.kMXP);

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // The speed of the robot in x and y translational velocities and rotational velocity
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Boolean statement to control locking the wheels in an X-position
    private boolean wheelsLocked = false;

    // Boolean statement to control autobalance functionality
    private boolean autoBalanceOn = false;

    private SwerveDriveOdometry odometry;

    private Pose2d pose = new Pose2d();

    public Drivetrain() {
        frontLeftModule = new SwerveModule(
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = new SwerveModule(
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = new SwerveModule(
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = new SwerveModule(
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        // TODO: We never needed to wait to zero the gyro before, why would we now...
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroGyroscope();
                // FIXME: Confirm this is correct
                odometry = new SwerveDriveOdometry(
                    KINEMATICS, getGyroscopeRotation(),
                    new SwerveModulePosition[] {
                      frontLeftModule.getPosition(),
                      frontRightModule.getPosition(),
                      backLeftModule.getPosition(),
                      backRightModule.getPosition()
                    });
                   SmartDashboard.putNumber("Pose X", pose.getX());
                   SmartDashboard.putNumber("Pose Y", pose.getY());
                   SmartDashboard.putNumber("Pose R", pose.getRotation().getDegrees());
                } catch (Exception e) {}
        }).start();
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        navx.reset();
    }

    /**
     * Toggles whether or not the wheels are locked. If they are, the wheels are
     * crossed into an X pattern and any other drive input has no effect.
     */
    public void toggleWheelsLocked() {
        wheelsLocked = !wheelsLocked;
    }

    /**
     * Toggles whether or not the robot is in autobalance mode. If it is, the robot
     * tries to zero out any roll or pitch messurements, and any driver input has no
     * effect.
     */
    public void toggleAutoBalance() {
        autoBalanceOn = !autoBalanceOn;
    }

    public double getGyroscopeAngle() {
        return Math.IEEEremainder(navx.getAngle(), 360.);
    }

    // Returns the measurment of the gyroscope yaw. Used for field-relative drive
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeAngle());
    }

    /**
     * Used to drive the robot with the provided ChassisSpeed object. However, if
     * the robot is in autobalance mode, the ChassisSpeed object is ignored, and a
     * new one is calculated based off the pitch and roll of the robot.
     * 
     * @param chassisSpeeds The translational and rotational velocities at which to
     *                      drive the robot.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        if (!autoBalanceOn) {
            // If we're not in autobalance mode, act normally.
            this.chassisSpeeds = chassisSpeeds;
        } else {
            // If we are in autobalance mode, calculate a new ChassisSpeed object based off
            // the measured pitch and roll
            this.chassisSpeeds = new ChassisSpeeds(
                    // X velocity is proportional to the sin of the roll angle
                    -MathUtil.applyDeadband(Math.sin(Math.toRadians(navx.getRoll())), Math.toRadians(0.5))
                            * MAX_VELOCITY_METERS_PER_SECOND * AUTOBALANCE_SPEED_FACTOR,
                    // Y velocity is proportional to the sin of the pitch angle
                    -MathUtil.applyDeadband(Math.sin(Math.toRadians(navx.getPitch())), Math.toRadians(0.5))
                            * MAX_VELOCITY_METERS_PER_SECOND * AUTOBALANCE_SPEED_FACTOR,
                    // No rotational velocity
                    0.0);
        }
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * This method is used to command the individual module states based off the
     * ChassisSpeeds object
     */
    @Override
    public void periodic() {
       SmartDashboard.putNumber("Pose X", pose.getX());
       SmartDashboard.putNumber("Pose Y", pose.getY());
       SmartDashboard.putNumber("Pose R", pose.getRotation().getDegrees());

        // Convert from ChassisSpeeds to SwerveModuleStates
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        // Make sure no modules are being commanded to velocites greater than the max possible velocity
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        if (!wheelsLocked) {
            // If we are not in wheel's locked mode, set the states normally
            frontLeftModule.setState(states[0]);
            frontRightModule.setState(states[1]);
            backLeftModule.setState(states[2]);
            backRightModule.setState(states[3]);
        } else {
            // If we are in wheel's locked mode, set the drive velocity to 0 so there is no
            // movment, and command the steer angle to either plus or minus 45 degrees to
            // form an X pattern.
            frontLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            frontRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            backRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        }

        pose = odometry.update(getGyroscopeRotation(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(), frontRightModule.getPosition(),
                        backLeftModule.getPosition(), backRightModule.getPosition()});
    }
}
