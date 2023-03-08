package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

/**
 * This command is used to drive the robot with a coordinate system that is
 * relative to the field, not the robot
 */
public class FieldOrientedDriveCommand extends CommandBase {
    // private final Drivetrain drivetrain;
    private final Drivetrain drivetrain;

    // DoubleSupplier objects need to be used, not double
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    public FieldOrientedDriveCommand(Drivetrain drivetrain,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        // Command requires the drivetrain subsystem
        addRequirements(drivetrain);
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * Send the input x, y, and rotation velocities to the drivetrain's drive method
     * as a ChassisSpeed object.
     */
    @Override
    public void execute() {
        // xVel = translationXSupplier.getAsDouble();
        // yVel = translationYSupplier.getAsDouble();
        // vel = Math.sqrt(Math.pow(xVel, 2.) + Math.pow(yVel, 2.));
        // alpha = Math.atan2(yVel, xVel);
        // velPrime = translationLimiter.calculate(vel);
        // xVelPrime = vel * Math.cos(alpha);
        // yVelPrime = vel * Math.sin(alpha);

        // SmartDashboard.putNumber("xVel", xVel);
        // SmartDashboard.putNumber("xVelPrime", xVelPrime);
        // SmartDashboard.putNumber("yVel", yVel);
        // SmartDashboard.putNumber("yVelPrime", yVelPrime);
        // SmartDashboard.putNumber("vel", vel);
        // SmartDashboard.putNumber("velPrime", velPrime);
        // SmartDashboard.putNumber("alpha", Math.toDegrees(alpha));
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(),
                rotationSupplier.getAsDouble(),
                drivetrain.getGyroscopeRotation()));
    }

    /** When the drive method is interupted, set all velocities to zero. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}
