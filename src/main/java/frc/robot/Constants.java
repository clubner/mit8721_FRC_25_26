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
    /* IP (v4) addresses
     * Reminder of static IP (v4) addresses used
     * The radio is automatically set to ______________.
     * The radio configuration IP is _________________.
     * The Access Point radio WPA is _____________.
     * The RoboRio is set to static ___________, mask ______________.
     */
    
    //SparkMax CAN IDs
    public static class kCan {
        // "B" is "back," "F" is "front," "L" is "left," "R" is "right," "T" is "turning," "D" is "driving"
        public static final int BLT_MOTOR = 1;
        public static final int BLD_MOTOR = 2;
        public static final int FLT_MOTOR = 3;
        public static final int FLD_MOTOR = 4;
        public static final int FRT_MOTOR = 5;
        public static final int FRD_MOTOR = 6;
        public static final int BRT_MOTOR = 7;
        public static final int BRD_MOTOR = 8;
        //Assignment for Pigeon 2.0
        public static final int PIDGEOTTO = 9;

    }
    
    //Joystick, controller, axis IDs
    //The "k" in front of "Controls" is a programming convention used to indicate a constant.
    public static class kControls {
        //Placeholders
        public static final int RIGHT_JOYSTICK = 1;
        public static final int LEFT_JOYSTICK = 2;
        public static final int COPILOT_GAMEPAD = 3;
        public static final int BUTTON_BOX = 4;
        public static final int MAIN_JOYSTICK = 5;

        
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
        public static final double MAX_LINEAR_SPEED = 3.0; // 3 meters per second, might also set to 4, 4.2, 4.4, 4.6, etc.
        public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation (or 180 degrees) per second
        public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; //Full rotation per second
        //Values for Swerve-specific geometry, constraints, & conversions
        //Invert the turning encoder because the output shaft rotates in the
        //opposite dirrection of the steering motor.
        public static final boolean TURNING_ENCODER_INVERTED = true;
        //Speeds of motors
        public static final double NEO_FREE_SPEED_RPM = 5676;
        //Dividing by 60 to get revs per second
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NEO_FREE_SPEED_RPM / 60;
        //Gear ratio & reduction conversions
        //Swerve Mk4i pinion gear tooth count
        public static final int PINION_TOOTH_COUNT = 14;
        // 45 teeth on wheel's bevel gear, 22 teeth on first-stage spur gear, 15 teeth on bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50) / (PINION_TOOTH_COUNT * 15 * 27);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE) / DRIVING_MOTOR_REDUCTION;
        // Driving encoder conversion calculations
        public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = WHEEL_CIRCUMFERENCE / DRIVING_MOTOR_REDUCTION; // meters per rotation
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_ROTATION = DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION / 60.0; //meters per second per rotation
        //Turning encoder reduction calculations
        public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0; // ratio between internal relative encoder and Thrifty absolute encoder
        //Turning encoder conversion calculations
        public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI) / TURNING_MOTOR_REDUCTION; //radians per rotation
        public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_SECOND_PER_ROTATION = TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION / 60.0; // radians per second per rotation
        //Limits for encoder
        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_ANGLE = 0; //radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_ANGLE = 2 * Math.PI; //radians
        //PID settings
        //Driving
        public static final double DRIVING_P = .04;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS; //feedforward settings, like feedback but proactive
        public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
        public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;
        //Turning
        public static final double TURNING_P = 1.0; //Might be too high, reduce if too much
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
        public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;




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
