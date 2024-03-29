package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;
import static frc.robot.Utilities.*;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here.
    private final Drivetrain drivetrain = new Drivetrain();

    // The drive team controllers are defined here.
    private final XboxController driverController = new XboxController(0);

    // Limits allowable acceleration
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rLimiter;

    // These variables limit the max speed of the robot
    private double speedModXY = .2;
    private double speedModR = .2;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Limits allowable acceleration
        xLimiter = new SlewRateLimiter(1. / TIME_TO_MAX_X);
        yLimiter = new SlewRateLimiter(1. / TIME_TO_MAX_Y);
        rLimiter = new SlewRateLimiter(1. / TIME_TO_MAX_R);

        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain,
            // () -> -modifyAxis(xLimiter.calculate(driverController.getLeftY())) * MAX_VELOCITY_METERS_PER_SECOND * speedModXY,
            // () -> -modifyAxis(yLimiter.calculate(driverController.getLeftX())) * MAX_VELOCITY_METERS_PER_SECOND * speedModXY,
            // () -> -modifyAxis(rLimiter.calculate(driverController.getRightX())) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * speedModR));
            () -> -modifyAxis(driverController.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND * speedModXY,
            () -> -modifyAxis(driverController.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND * speedModXY,
            () -> -modifyAxis(driverController.getRightX()) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * speedModR));

        // Configure the button bindings
        configureButtonBindings();
    }

    /** Button -> command mappings are defined here. */
    private void configureButtonBindings() {
        // Driver Controller Bindings
        // A button zeros the gyroscope
        new Trigger(driverController::getAButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyroscope()));
        // // X button toggles the wheel locks
        // new Trigger(driverController::getXButton).onTrue(new InstantCommand(() -> drivetrain.toggleWheelsLocked()));
        // // Y button toggles autobalance mode
        // new Trigger(driverController::getYButton).onTrue(new InstantCommand(() -> drivetrain.toggleAutoBalance()));

        new Trigger(driverController::getYButton).whileTrue(drivetrain.driveToTarget(() -> 1., () -> 0., () -> 0.));
        // new Trigger(driverController::getAButton).whileTrue(drivetrain.driveToTarget(() -> -1., () -> 0., () -> 0.));
        new Trigger(driverController::getXButton).whileTrue(drivetrain.driveToTarget(() -> 0., () -> 1., () -> 0.));
        new Trigger(driverController::getBButton).whileTrue(drivetrain.driveToTarget(() -> 0., () -> -1., () -> 0.));

        new Trigger(driverController::getBButton).and(driverController::getLeftBumper).onTrue(new InstantCommand(() -> this.speedDown()));
        new Trigger(driverController::getBButton).and(driverController::getRightBumper).onTrue(new InstantCommand(() -> this.speedUp()));
    }

    private void speedDown() {
        // speedModXY = MathUtil.clamp(speedModXY * (3/4), 0, 1);
        // speedModR = MathUtil.clamp(speedModR * (3/4), 0, 1);
        speedModXY = .2;
        speedModR = .2;
    }

    private void speedUp() {
        // speedModXY = MathUtil.clamp(speedModXY * (4/3), 0, 1);
        // speedModR = MathUtil.clamp(speedModR * (4/3), 0, 1);
        speedModXY = .8;
        speedModR = .8;

    }

    public Command getAutonomousCommand() {
        return null;
        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        //         0.5, // kMaxSpeedMetersPerSecond,
        //         0.5) // kMaxAccelerationMetersPerSecondSquared)
        //         // Add kinematics to ensure max speed is actually obeyed
        //         .setKinematics(KINEMATICS);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(new Translation2d(1, .75),
        //                 new Translation2d(3, 0),
        //                 new Translation2d(2, -0.75)),
        //         new Pose2d(0.64, -0.07, new Rotation2d(3.14)),
        //         config);

        // TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        //     Math.PI, // kMaxAngularSpeedRadiansPerSecond,
        //     Math.PI); // kMaxAngularSpeedRadiansPerSecondSquared);
                
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //         1., // AutoConstants.kPThetaController,
        //         0,
        //         0,
        //         kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //         exampleTrajectory,
        //         drivetrain::getPose, // Functional interface to feed supplier
        //         KINEMATICS,
        //         // Position controllers
        //         new PIDController(0.85, 0, 0),
        //         new PIDController(0.85, 0, 0),
        //         thetaController,
        //         drivetrain::setModuleStates,
        //         drivetrain);

        // // Reset odometry to the starting pose of the trajectory.
        // drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> drivetrain.drive(new ChassisSpeeds(0, 0, 0)));
    }
}
