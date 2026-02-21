// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.AnalogEncoder;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
//import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;





public class SwerveModule {
  
  //Commented out below to delete later, reminder for now (moved to constants)
  //private static final double kWheelRadius = 0.0508;
  //Commented out below to delete later, reminder for now (moved to constants)
  
  //Unsure if I need this - it's already in the constants folder
  //private static final double kModuleMaxAngularVelocity = Constants.kSwerve.MAX_ANGULAR_SPEED;
  //private static final double kModuleMaxAngularAcceleration =
    //2 * Math.PI; // radians per second squared

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turnConfig;

    private final RelativeEncoder driveEncoder;
    private final AnalogEncoder turnEncoder;

    private final SparkClosedLoopController driveClosedLoopController;
    private final SparkClosedLoopController turnClosedLoopController;

 
    private double offset;
    private SwerveModuleState swerveState = new SwerveModuleState(0.0, new Rotation2d());
    /*
    
    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.kSwerve.MAX_ANGULAR_SPEED, Constants.kSwerve.MAX_ANGULAR_ACCELERATION));

  // Gains are for example purposes only - must be determined for your own robot!
  //Figure out how to test this
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);


     */
    

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, turning encoder, and encoder offset.
   *
   * @param driveMotorID SparkMax Driving Motor CAN ID
   * @param turnMotorID SparkMax Turning Motor CAN ID
   * @param encoderID DIO/PWM channel for Thrifty encoder
   * @param encoderOffset Encoder offset
   */
  public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, double encoderOffset) {
    //Creating and configuring motors
    //Drive motor
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    driveConfig = new SparkMaxConfig();
    driveConfig.idleMode(Constants.kSwerve.DRIVING_MOTOR_IDLE_MODE).smartCurrentLimit(Constants.kSwerve.DRIVING_MOTOR_CURRENT_LIMIT);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Turn motor
    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
    turnConfig = new SparkMaxConfig();
    turnConfig.idleMode(Constants.kSwerve.TURNING_MOTOR_IDLE_MODE).smartCurrentLimit(Constants.kSwerve.TURNING_MOTOR_CURRENT_LIMIT);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Encoders
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = new AnalogEncoder(encoderID);
    //Closed Loop Controllers
    driveClosedLoopController = driveMotor.getClosedLoopController();
    turnClosedLoopController = turnMotor.getClosedLoopController();
    //Offset settings
    offset = encoderOffset;
    swerveState.angle = new Rotation2d(turnEncoder.get());
    driveEncoder.setPosition(0);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * Constants.kSwerve.WHEEL_RADIUS / Constants.kEncoders.ENCODER_RESOLUTION);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / Constants.kEncoders.ENCODER_RESOLUTION);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      driveEncoder.getVelocity(), 
      new Rotation2d(turnEncoder.get() - offset));
  }


  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), 
        new Rotation2d(turnEncoder.get() - offset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    //Apply chassis offset to desired state
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(offset));
    
    var encoderRotation = new Rotation2d(turnEncoder.get());
    
    //Optimize the reference state to avoid spinning more than 90 degrees
    correctedDesiredState.optimize(encoderRotation);
    //Cosine softening
    correctedDesiredState.cosineScale(encoderRotation);

    //Closed Loop Controllers
    driveClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    
    

    swerveState = correctedDesiredState;
    // Calculate the drive output from the drive PID controller.
  }
  // Zero SwerveModule drive encoders
  public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }
  
  // Return turn encoder angle
  public double getAngle() {
    return turnEncoder.get();
  }

}
