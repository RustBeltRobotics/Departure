package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Maximum battery voltage */
    public static final double MAX_VOLTAGE = 12.;

    // Spark Max current limits
    /** Smart drive motor current */
    public static final int DRIVE_SMART_CURRENT_LIMIT = 30;
    /** Smart steer motor current */
    public static final int STEER_SMART_CURRENT_LIMIT = 20;
    /** Secondary drive motor current */
    public static final int DRIVE_SECONDARY_CURRENT_LIMIT = 80;
    /** Secondary steer motor current */
    public static final int STEER_SECONDARY_CURRENT_LIMIT = 80;

    /**
     * If the robot passes from greater than this to less than this, the robot is
     * considered balanced on top of the charge station
     */
    public static final double CHARGE_STATION_BALANCED_ANGLE = 0.5;
    
    /**
     * If the robot passes from less than this to greater than this, the robot is
     * considered to have started driving up the ramp
     */
    public static final double CHARGE_STATION_UNBALANCED_ANGLE = 5.;

    // Drivetrain Constants
    /**
     * The left-to-right distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.47;

    /**
     * The front-to-back distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.47;

    /**
     * Creates a swerve kinematics object, to convert desired chassis velocity into
     * individual module states
     */
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    /** Conversion between rotations and meters */
    public static final double DRIVE_POSITION_CONVERSION = Math.PI
            * SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
            * SdsModuleConfigurations.MK4I_L2.getDriveReduction();

    /** Conversion between rotations per minute and meters per seconds */
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.;

    /**
     * The maximum linear velocity of the robot in meters per second. This is a
     * measure of how fast the robot can move linearly.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676. * DRIVE_VELOCITY_CONVERSION;

    /**
     * The maximum angular velocity of the robot in radians per second. This is a
     * measure of how fast the robot can rotate in place.
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    /** Conversion between rotations and degrees */
    public static final double STEER_POSITION_CONVERSION = 360. * SdsModuleConfigurations.MK4I_L2.getSteerReduction();

    /** Conversion between rotations per minute and degrees per seconds */
    public static final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.;

    /** This factor is applied to the maximum drive velocity during autobalancing */
    public static final double AUTOBALANCE_SPEED_FACTOR = 0.25;

    /** Time (seconds) to get to max x velocity under manual control */
    public static final double TIME_TO_MAX_X = .1;
    /** Time (seconds) to get to max y velocity under manual control */
    public static final double TIME_TO_MAX_Y = .1;
    /** Time (seconds) to get to max rotational velocity under manual control */
    public static final double TIME_TO_MAX_R = .1;

    // Steer PID Constants
    public static final double STEER_P = 0.008;
    public static final double STEER_I = 0.;
    public static final double STEER_D = 0.0002;

    // CAN ID
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 15;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 16;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 32;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 12;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 34;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 17;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 18;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 31;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 14;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 33;

    // Module Offsets - rotational offsets such that the modules all read 0 degress when facing forward
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -267.89061970149197;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -127.88085318858336;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -84.55078196199361;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.878906234437155;
}
