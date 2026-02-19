package frc.robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;




/*
 * This class contains values that remain constant while the robot is running.
 * It's divided in to subclasses in an effort to improve organization and readability.
 * Comments have been supplied to further improve clarity.
 */

 public class Constants {
    //Joystick, controller, axis IDs
    //The "k" in front of "Controls" is a programming convention used to indicate a constant.
    public static class kControls {
        //We'll figure this out when we start connecting it with a controller & joystick
    }

    //Constants for the swerve drive
    public static class kSwerve {
        //Drive train geometry constants
        public static final double DT_WIDTH = Units.inchesToMeters(22.25); //Width of drivetrain from middle of wheels
        public static final double DT_LENGTH = Units.inchesToMeters(22.25); //Length of drivetrain 
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double WHEEL_RADIUS = Constants.kSwerve.WHEEL_DIAMETER / 2.0;
        public static final double WHEEL_CIRCUMFERENCE = Constants.kSwerve.WHEEL_DIAMETER * Math.PI;
        //Drive train linear velocity/angular velocity/linear acceleration/angular acceleration constants
        public static final double MAX_LINEAR_SPEED = 3.0; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation (or 180 degrees) per second
        public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; //Full rotation per second

        //Setup for swerve drive kinematics
        //"Translation2d" orients the four wheels around the robot's center
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DT_LENGTH / 2.0, DT_WIDTH / 2.0), //Front left wheel
            new Translation2d(DT_LENGTH / 2.0, -DT_WIDTH / 2.0), //Back left wheel
            new Translation2d(-DT_LENGTH / 2.0, DT_WIDTH / 2.0), //Back right wheel
            new Translation2d(-DT_LENGTH / 2.0, -DT_WIDTH / 2.0) //Front right wheel
        );

    }

    public static class kEncoders {
        public static final int ENCODER_RESOLUTION = 4096; //Set to 4096 because ThriftyBot Absolute Magnetic Encoder features 12-bit resolution

    }
 }
