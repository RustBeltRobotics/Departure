// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.subsystems.Drivetrain;

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

    private double speedModXY = 0.25;
    private double speedModR = 0.25;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain,
            () -> -Utilities.modifyAxis(driverController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND * speedModXY,
            () -> -Utilities.modifyAxis(driverController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND * speedModXY,
            () -> -Utilities.modifyAxis(driverController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * speedModR));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Driver Controller Bindings
        // A button zeros the gyroscope
        new Trigger(driverController::getAButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyroscope()));
        // X button toggles the wheel locks
        new Trigger(driverController::getXButton).onTrue(new InstantCommand(() -> drivetrain.toggleWheelsLocked()));
        // Y button toggles autobalance mode
        new Trigger(driverController::getYButton).onTrue(new InstantCommand(() -> drivetrain.toggleAutoBalance()));

        new Trigger(driverController::getBButton).and(driverController::getLeftBumper).onTrue(new InstantCommand(() -> this.speedDown()));
        new Trigger(driverController::getBButton).and(driverController::getRightBumper).onTrue(new InstantCommand(() -> this.speedUp()));
    }

    private void speedDown() {
        speedModXY = Utilities.clamp(speedModXY * (3/4), 0, 1);
        speedModR = Utilities.clamp(speedModR * (3/4), 0, 1);
    }

    private void speedUp() {
        speedModXY = Utilities.clamp(speedModXY * (4/3), 0, 1);
        speedModR = Utilities.clamp(speedModR * (4/3), 0, 1);
    }
}
