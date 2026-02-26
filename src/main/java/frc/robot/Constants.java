package frc.robot;

import static edu.wpi.first.apriltag.AprilTagFields.kDefaultField;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;
import java.io.IOException;
import java.util.HashMap;
import edu.wpi.first.wpilibj.RobotBase;

//import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;




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
        //Ports
        public static final int PRIMARY_PORT = 0;
        public static final int SECONDARY_PORT = 1; //possibly unnecessary
        public static final double DRIVE_DEADBAND = .05;
        //Placeholders for buttons
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
        //Reverse Pigeon gyro if it's inverted
        public static final boolean PIDGEOTTO_INVERTED = false;
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

        //Idle modes
        public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

        //Current limits
        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 40; //amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; //amps

        







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
        public static final int FL_ENCODER = 0;
        public static final int FR_ENCODER = 3;
        public static final int BL_ENCODER = 1;
        public static final int BR_ENCODER = 2;
        public static final double FL_ENCODER_OFFSET = 0.0;
        public static final double FR_ENCODER_OFFSET = 0.0;
        public static final double BL_ENCODER_OFFSET = 0.0;
        public static final double BR_ENCODER_OFFSET = 0.0;

    }

    public static class kVision {

    }

    public static class kField {
        //Distance of field in inches
        public static final Distance FIELD_LENGTH = Inches.of(651.2225);
        public static final Distance FIELD_WIDTH = Inches.of(317.6875);
        //Measurements of center bump
        public static final Distance BUMP_WIDTH = Inches.of(73.08122);
        public static final Distance BUMP_DEPTH = Inches.of(47);
        public static final Distance BUMP_TO_WALL = Inches.of(62.37375);
        public static final Distance BUMP_TO_DRIVER_STATION = Inches.of(182.11125);
        //Measurement of trench width
        public static final Distance TRENCH_WIDTH = Inches.of(50.34375);
        //Measurement from center of bumper/trench to field center
        public static final Distance BUMP_CENTER_Y_TO_FIELD_CENTER = 
            FIELD_WIDTH.div(2).minus(BUMP_TO_WALL).plus(BUMP_WIDTH.div(2));
        public static final Distance TRENCH_CENTER_Y_TO_FIELD_CENTER = 
            FIELD_WIDTH.div(2).minus(TRENCH_WIDTH.div(2));
        //Starting line distance
        public static final Distance STARTING_LINE_DISTANCE = Inches.of(158.6);
        //Hub centers
        public static final Translation2d HUB_CENTER_BLUE = 
            new Translation2d(Inches.of(182.11125), FIELD_WIDTH.div(2));
        public static final Translation2d HUB_CENTER_RED = 
            new Translation2d(Inches.of(469.11125), FIELD_WIDTH.div(2));
        //Depot measurements
        public static final Distance DEPOT_WIDTH = Inches.of(42);
        public static final Distance DEPOT_LENGTH = Inches.of(24);
        public static final Distance DEPOT_TO_WALL = Inches.of(213.84375);
        public static final Distance DEPOT_CENTER_TO_WALL = DEPOT_TO_WALL.plus(DEPOT_WIDTH.div(2));
        public static final Distance OUTPOST_CENTER_TO_WALL = Inches.of(47.5).div(2);
        //Robot length
        public static final Distance ROBOT_LENGTH = Inches.of(kSwerve.DT_LENGTH);

        //April tags - select "simulation"/"real" to decide what mode to function in
        //private static String fieldMode = "real";
        public static final AprilTagFieldLayout APRIL_TAGS = AprilTagFieldLayout.loadField(kDefaultField);
        //public static final AprilTagFieldLayout APRIL_TAGS = (fieldMode.equals("simulation")) 
            //? AprilTagFieldLayout.loadFromResource(kDefaultField.m_resourceFile) : AprilTagFieldLayout.loadField(kDefaultField);
        

        //Add a new value and automatically adds it to the auto chooser (I don't really understand this part)
        public static HashMap<String, Translation2d> AutoConstants() {
            HashMap<String, Translation2d> points = new HashMap<>();
            points.put(
                "Bump REn, ", 
                new Translation2d(STARTING_LINE_DISTANCE.minus(BUMP_DEPTH.div(2)),
                    BUMP_WIDTH.div(2).plus(BUMP_TO_WALL)));
            points.put(
                "Bump REx, ",
                new Translation2d(STARTING_LINE_DISTANCE.plus(BUMP_DEPTH).plus(ROBOT_LENGTH.div(2)),
                    BUMP_WIDTH.div(2).plus(BUMP_TO_WALL)));
            points.put(
                "Bump LEn, ",
                new Translation2d(STARTING_LINE_DISTANCE.minus(BUMP_DEPTH.div(2)),
                    FIELD_WIDTH.minus(BUMP_WIDTH.div(2).plus(BUMP_TO_WALL))));
            points.put(
                "Bump LEx, ",
                new Translation2d(STARTING_LINE_DISTANCE.plus(BUMP_DEPTH).plus(ROBOT_LENGTH.div(2)),
                    FIELD_WIDTH.minus(BUMP_WIDTH.div(2).plus(BUMP_TO_WALL))));
            return points;
        }
        //Creating bump enumeration
        public class Bump {
            public enum BumpLocation {
                BLUE_LEFT(
                    new Translation2d(
                        BUMP_TO_DRIVER_STATION,
                        FIELD_WIDTH.div(2).plus(BUMP_CENTER_Y_TO_FIELD_CENTER.minus(BUMP_WIDTH.div(2)))
                        ),
                    new Translation2d(
                        BUMP_TO_DRIVER_STATION,
                        FIELD_WIDTH.div(2).plus(BUMP_CENTER_Y_TO_FIELD_CENTER.plus(BUMP_WIDTH.div(2)))
                    )
                    ),
                BLUE_RIGHT(
                    new Translation2d(
                        BUMP_TO_DRIVER_STATION,
                        FIELD_WIDTH.div(2).minus(BUMP_CENTER_Y_TO_FIELD_CENTER.minus(BUMP_WIDTH.div(2)))
                        ),
                    new Translation2d(
                        BUMP_TO_DRIVER_STATION,
                        FIELD_WIDTH.div(2).minus(BUMP_CENTER_Y_TO_FIELD_CENTER.plus(BUMP_WIDTH.div(2)))
                    )
                    ),
                RED_RIGHT(
                    new Translation2d(
                        FIELD_LENGTH.minus(BUMP_TO_DRIVER_STATION),
                        FIELD_WIDTH.div(2).plus(BUMP_CENTER_Y_TO_FIELD_CENTER.minus(BUMP_WIDTH.div(2)))
                        ),
                    new Translation2d(
                        FIELD_LENGTH.minus(BUMP_TO_DRIVER_STATION),
                        FIELD_WIDTH.div(2).plus(BUMP_CENTER_Y_TO_FIELD_CENTER.plus(BUMP_WIDTH.div(2)))
                    )
                    ),
                RED_LEFT(
                    new Translation2d(
                        FIELD_LENGTH.minus(BUMP_TO_DRIVER_STATION),
                        FIELD_WIDTH.div(2).minus(BUMP_CENTER_Y_TO_FIELD_CENTER.minus(BUMP_WIDTH.div(2)))
                        ),
                    new Translation2d(
                        BUMP_TO_DRIVER_STATION,
                        FIELD_WIDTH.div(2).minus(BUMP_CENTER_Y_TO_FIELD_CENTER.plus(BUMP_WIDTH.div(2)))
                    )
                    );
            public final Translation2d translationInside;
            public final Translation2d translationOutisde;
            public final Translation2d average;
            //Constructor for bump location enum
            BumpLocation(Translation2d translationInside, Translation2d translationOutside) {
                this.translationInside = translationInside;
                this.translationOutisde = translationOutside;
                this.average = translationInside.plus(translationOutisde).div(2);
            }
            
            public static BumpLocation getClosest(Translation2d translation) {
                double closestDistance = Double.MAX_VALUE;
                BumpLocation closestBumpLocation = BLUE_LEFT;

                for (BumpLocation bumpLocation : BumpLocation.values()) {
                    double distance = translation.getDistance(bumpLocation.translationInside);
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestBumpLocation = bumpLocation;
                    }
                }
                return closestBumpLocation;
            }
                
            }
        }



    }
 }
