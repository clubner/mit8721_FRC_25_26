// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kCan;
import frc.robot.Constants.kControls;
import frc.robot.Constants.kEncoders;
import frc.robot.Constants.kSwerve;

import com.ctre.phoenix6.hardware.Pigeon2;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  //Moved to Constants folder, changed to values for our robot's width/height
  //private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  //private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  //private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  //private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule frontLeftModule = new SwerveModule(kCan.FLD_MOTOR, kCan.FLT_MOTOR, kEncoders.FL_ENCODER, kEncoders.FL_ENCODER_OFFSET);
  private final SwerveModule frontRightModule = new SwerveModule(kCan.FRD_MOTOR, kCan.FRT_MOTOR, kEncoders.FR_ENCODER, kEncoders.FR_ENCODER_OFFSET);
  private final SwerveModule backLeftModule = new SwerveModule(kCan.BLD_MOTOR, kCan.BLT_MOTOR, kEncoders.BL_ENCODER, kEncoders.BL_ENCODER_OFFSET);
  private final SwerveModule backRightModule = new SwerveModule(kCan.BRD_MOTOR, kCan.BRT_MOTOR, kEncoders.BR_ENCODER, kEncoders.BR_ENCODER_OFFSET);

  //Will need to update
  private final Pigeon2 pidgeotto = new Pigeon2(kCan.PIDGEOTTO);
  //Already did this in Constants file
  //private final SwerveDriveKinematics m_kinematics =
      //new SwerveDriveKinematics(
         // m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          Constants.kSwerve.KINEMATICS,
          pidgeotto.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    pidgeotto.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        Constants.kSwerve.KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, pidgeotto.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        pidgeotto.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
