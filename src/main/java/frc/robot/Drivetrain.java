// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCan;
//import frc.robot.Constants.kControls;
import frc.robot.Constants.kEncoders;
import frc.robot.Constants.kSwerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


import com.ctre.phoenix6.hardware.Pigeon2;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  
  //public static final double kMaxSpeed = 3.0; // 3 meters per second
  //public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  //Moved to Constants folder, changed to values for our robot's width/height
  //private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  //private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  //private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  //private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule frontLeftModule = new SwerveModule(kCan.FLD_MOTOR, kCan.FLT_MOTOR, kEncoders.FL_ENCODER, kEncoders.FL_ENCODER_OFFSET);
  private final SwerveModule frontRightModule = new SwerveModule(kCan.FRD_MOTOR, kCan.FRT_MOTOR, kEncoders.FR_ENCODER, kEncoders.FR_ENCODER_OFFSET);
  private final SwerveModule backLeftModule = new SwerveModule(kCan.BLD_MOTOR, kCan.BLT_MOTOR, kEncoders.BL_ENCODER, kEncoders.BL_ENCODER_OFFSET);
  private final SwerveModule backRightModule = new SwerveModule(kCan.BRD_MOTOR, kCan.BRT_MOTOR, kEncoders.BR_ENCODER, kEncoders.BR_ENCODER_OFFSET);

  //Gyro sensor - Pigeon 2
  private final Pigeon2 pidgeotto = new Pigeon2(kCan.PIDGEOTTO);
  //Already did this in Constants file
  //private final SwerveDriveKinematics m_kinematics =
      //new SwerveDriveKinematics(
         // m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          Constants.kSwerve.KINEMATICS,
          pidgeotto.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
          });
  //Constructor
  public Drivetrain() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    }
    catch (Exception e) {
      e.printStackTrace();
      return;
    }
    
    AutoBuilder.configure(
      this::getPose, //Robot pose supplier
      this::resetOdometry, //method to reset odometry
      this::getRobotRelativeSpeeds, //ChassisSpeeds supplier, MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), //method to drive robot given ROBOT RELATIVE ChassisSpeeds
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0)
      ),
      config, //Robot configuration
      () -> {
        //Boolean supplier that controls when path is mirrored for red alliance
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this //Reference to this subsystem to set requirements
    );
    //pidgeotto.reset();
  }
  
  

  //Method to configure autonomous
  public void configAuto() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    }
    catch (Exception e) {
      e.printStackTrace();
      return;
    }
    //Configure AutoBuilder last
    AutoBuilder.configure(
      this::getPose, //Robot pose supplier
      this::resetOdometry, //method to reset odometry
      this::getRobotRelativeSpeeds, //ChassisSpeeds supplier, MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), //method to drive robot given ROBOT RELATIVE ChassisSpeeds
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0)
      ),
      config, //Robot configuration
      () -> {
        //Boolean supplier that controls when path is mirrored for red alliance
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this //Reference to this subsystem to set requirements
    );
  }
  //Getter method for robot relative speeds
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return kSwerve.KINEMATICS.toChassisSpeeds(
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState());
  }

  //Method to drive robot at speeds relative to chassis
  private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  //Overriding periodic method
  @Override
  public void periodic() {
    odometry.update(
      pidgeotto.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      });
    
    SmartDashboard.putNumber("Front Left", frontLeftModule.getAngle());
    SmartDashboard.putNumber("Front Right", frontRightModule.getAngle());
    SmartDashboard.putNumber("Back Left", backLeftModule.getAngle());
    SmartDashboard.putNumber("Back Right", backRightModule.getAngle());
  }

  //Getter for pose
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  //Setter for resetting odometry
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      pidgeotto.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      },
      pose);
  }
  
  /*
   * The main drive method
   */

  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * kSwerve.MAX_LINEAR_SPEED;
    double ySpeedDelivered = ySpeed * kSwerve.MAX_LINEAR_SPEED;
    double rotDelivered = rotation * kSwerve.MAX_ANGULAR_SPEED;

    var swerveModuleStates = kSwerve.KINEMATICS.toSwerveModuleStates(
      fieldRelative 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, 
          pidgeotto.getRotation2d())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kSwerve.MAX_LINEAR_SPEED);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  //Method for setting the wheels into an X formation
  public void setX() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  //Method for setting module states
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kSwerve.MAX_LINEAR_SPEED);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);

  }

  //Method for resetting encoders
  public void resetEncoders() {
    frontLeftModule.resetDriveEncoder();
    backLeftModule.resetDriveEncoder();
    frontRightModule.resetDriveEncoder();
    backRightModule.resetDriveEncoder();
  }

  //Zero Pigeon 2.0
  public void zeroHeading() {
    pidgeotto.reset();
  }

  //Getter method for current heading in degrees
  public double getHeading() {
    return pidgeotto.getRotation2d().getDegrees();
  }

  //Getter method for turn rate
  public double getTurnRate() {
    return pidgeotto.getAngularVelocityZWorld().getValueAsDouble() * (kSwerve.PIDGEOTTO_INVERTED ? -1.0 : 1.0);
  }

}







