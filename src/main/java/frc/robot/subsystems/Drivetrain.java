package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
    // NavX connected over MXP
    public final AHRS navx;

    private double gyroOffset;

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

    // Odometry object for the drivetrain
    private SwerveDriveOdometry odometry;

    // PID controllers used for driving to a commanded x/y/r location
    private PIDController xController;
    private PIDController yController;
    private PIDController rController;

    public Drivetrain() {
        // Initialize all modules
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

        // Zero all relative encoders
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();

        // Initialize and zero gyro
        navx = new AHRS(SPI.Port.kMXP);
        zeroGyroscope();

        // Initialize odometry object
        odometry = new SwerveDriveOdometry(
                KINEMATICS, getGyroscopeRotation(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(),
                        backRightModule.getPosition()
                });
        
        // Initialize PID Controllers
        // TODO: Move constants to constants file
        xController = new PIDController(1., 0., 0.);
        yController = new PIDController(1., 0., 0.);
        rController = new PIDController(.1, 0., 0.);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyroOffset = -getGyroscopeAngle();
        // navx.reset();
    }

    public double getGyroOffset() {
        return gyroOffset;
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
        return Math.IEEEremainder(360. - navx.getAngle(), 360.);
    }

    // Returns the measurment of the gyroscope yaw. Used for field-relative drive
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeAngle());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                getGyroscopeRotation(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(),
                        backRightModule.getPosition()
                },
                pose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);
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
     * Uses a feedforward controller and a PID controller to drive the arm to a
     * commanded extension. Sets the rotation rate to zero to keep rotation and
     * extension separate.
     * 
     * @param extension The extension to move the arm to. Fully retracted is 0,
     *                  positive is exteneded.
     * @return The command for driving to the desired extension.
     */
    public Command driveToTarget(DoubleSupplier xTarget, DoubleSupplier yTarget, DoubleSupplier rTarget) {
        return new FunctionalCommand(
            // initialize(): reset PID controller and set setpoint
            () -> {
                xController.reset();
                yController.reset();
                rController.reset();
                xController.setSetpoint(xTarget.getAsDouble());
                yController.setSetpoint(yTarget.getAsDouble());
                rController.setSetpoint(rTarget.getAsDouble());
            },
            // execute(): drive robot with velocities calculated by PID controllers
            () -> {
                // Calculate velocities
                double xVel = xController.calculate(getPose().getX(), xTarget.getAsDouble());
                double yVel = yController.calculate(getPose().getY(), yTarget.getAsDouble());
                double rVel = rController.calculate(getGyroscopeAngle(), rTarget.getAsDouble());

                drive(new ChassisSpeeds(xVel, yVel, rVel));
            },
            // end(): set drive voltages to zero
            interupted -> drive(new ChassisSpeeds(0., 0., 0.)),
            // isFinished(): check if PID controllers are at setpoint
            () -> (xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint()),
            // Require the drivetrain subsystem
            this
        );
    }

    public Command driveToPose(Pose2d destination) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                0.5, // kMaxSpeedMetersPerSecond,
                0.5) // kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(KINEMATICS);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                getPose(),
                List.of(new Translation2d(
                    (getPose().getX() + destination.getX())/2,
                    (getPose().getY() + destination.getY())/2)),
                destination,
                config);

        TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            Math.PI, // kMaxAngularSpeedRadiansPerSecond,
            Math.PI); // kMaxAngularSpeedRadiansPerSecondSquared);
                
        ProfiledPIDController thetaController = new ProfiledPIDController(
                .01, // AutoConstants.kPThetaController,
                0,
                0,
                kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                this::getPose, // Functional interface to feed supplier
                KINEMATICS,
                // Position controllers
                new PIDController(0.85, 0, 0),
                new PIDController(0.85, 0, 0),
                thetaController,
                this::setModuleStates,
                this);

        // Reset odometry to the starting pose of the trajectory.
        // FIXME: I don't think we want to do this...

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> this.drive(new ChassisSpeeds(0, 0, 0)));
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * This method is used to command the individual module states based off the
     * ChassisSpeeds object
     */
    @Override
    public void periodic() {
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

        // Update the odometry
        odometry.update(
                getGyroscopeRotation(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(), frontRightModule.getPosition(),
                        backLeftModule.getPosition(), backRightModule.getPosition()
                });
    }
}
