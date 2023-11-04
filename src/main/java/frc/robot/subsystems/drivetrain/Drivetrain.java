// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.modules.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  // Create MAXSwerveModules
  private final SwerveModule frontLeft = SwerveModule.create(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset
  );

  private final SwerveModule frontRight = SwerveModule.create(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset
  );

  private final SwerveModule backLeft = SwerveModule.create(
      DriveConstants.kBackLeftDrivingCanId,
      DriveConstants.kBackLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset
  );

  private final SwerveModule backRight = SwerveModule.create(
      DriveConstants.kBackRightDrivingCanId,
      DriveConstants.kBackRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset
  );

  // The gyro sensor
  private final WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.kPigeon2CanId);
  private final BasePigeonSimCollection gyroSim = gyro.getSimCollection();

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  double fieldRelativeOffset = 0;

  // Pose Estimator class for tracking robot pose
  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(gyro.getAngle()),
    new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    }, 
    new Pose2d(new Translation2d(4,4), new Rotation2d())
  );  

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {

    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      fieldRelativeOffset = 180;
    }else
    {
      fieldRelativeOffset = 0;
    }

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param _pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d _pose) {
    
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      fieldRelativeOffset = 180;
    }else
    {
      fieldRelativeOffset = 0;
    }

    gyro.setYaw(_pose.getRotation().getDegrees());

    poseEstimator.resetPosition(
      _pose.getRotation(),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      },
      _pose
    );
  }

  public void updatePoseEstimator(){
    poseEstimator.update(
      Rotation2d.fromDegrees(-gyro.getAngle()),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }
    );

    SmartDashboard.putNumber("Gryo Angle", -gyro.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param _xSpeed        Speed of the robot in the x direction (forward).
   * @param _ySpeed        Speed of the robot in the y direction (sideways).
   * @param _rot           Angular rate of the robot.
   * @param _fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param _rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double _xSpeed, double _ySpeed, double _rot, boolean _fieldRelative, boolean _rateLimit) {

    //This method is for joystick values, so scale values just incase.
    _xSpeed = MathUtil.clamp(_xSpeed, -1, 1);
    _ySpeed = MathUtil.clamp(_ySpeed, -1, 1);
    _rot = MathUtil.clamp(_rot, -1, 1);

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (_rateLimit) {

      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(_ySpeed, _xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(_xSpeed, 2) + Math.pow(_ySpeed, 2));

      //Rate Limit the Magnitude
      currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      currentTranslationDir = inputTranslationDir;
      
      //Convert from polar back to XY
      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);

      currentRotation = rotLimiter.calculate(_rot);

    } else {
      xSpeedCommanded = _xSpeed;
      ySpeedCommanded = _ySpeed;
      currentRotation = _rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      _fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-gyro.getAngle() + fieldRelativeOffset))
      : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param _desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] _desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        _desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(_desiredStates[0]);
    frontRight.setDesiredState(_desiredStates[1]);
    backLeft.setDesiredState(_desiredStates[2]);
    backRight.setDesiredState(_desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void simulate(){
    SmartDashboard.putData("Module 1", frontLeft);
    SmartDashboard.putData("Module 2", frontRight);
    SmartDashboard.putData("Module 3", backLeft);
    SmartDashboard.putData("Module 4", backRight);
    gyroSim.addHeading(Units.radiansToDegrees(DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond) * 0.02);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState(),
      
    };
  }

}
