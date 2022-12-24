// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class Drivetrain extends SubsystemBase {
  
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  //module objects
  private SwerveModule frontLeft = new SwerveModule(1, 1, 5, ModuleConstants.MODULE1_OFFSET, true, 0); // Module 1
  private SwerveModule frontRight = new SwerveModule(2, 2, 6, ModuleConstants.MODULE2_OFFSET, false, 1); // Module 2
  private SwerveModule backLeft = new SwerveModule(3, 3, 7, ModuleConstants.MODULE3_OFFSET, true, 2); // Module 3
  private SwerveModule backRight = new SwerveModule(4, 4, 8, ModuleConstants.MODULE4_OFFSET, false, 3); // Module 4

  //module positions
  public final Translation2d frontLeftLocation = new Translation2d(0.175,0.175);
  public final Translation2d frontRightLocation = new Translation2d(0.175, -0.175);
  public final Translation2d backLeftLocation = new Translation2d(-0.175,0.175);
  public final Translation2d backRightLocation = new Translation2d(-0.175, -0.175);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  //odometer for measuring field position
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0));

  public boolean allModuleHomeStatus = false;

  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    getChassisAngle(), new Pose2d(), kinematics,
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02,0.02,0.02), 
    new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02), 
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1,0.1,0.1),
    0.02);

  public Drivetrain() {
    calibrateGyro();
  }

  @Override
  public void periodic() {

    odometer.update(getChassisAngle(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());

    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("1 Homed", frontLeft.homeFinished);
    SmartDashboard.putBoolean("2 Homed", frontRight.homeFinished);
    SmartDashboard.putBoolean("3 Homed", backLeft.homeFinished);
    SmartDashboard.putBoolean("4 Homed", backRight.homeFinished);

    SmartDashboard.putNumber("Bot Heading", getChassisAngle().getDegrees());
    SmartDashboard.putString("Bot Position", getPose().getTranslation().toString());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Drive the bot using joystick input.
   * Positive x is forward, y - left, rot - CC rotation
   * 
   * 
   * @param xSpeed m/s speed positive away from drive station
   * @param ySpeed m/s speed positive left of drive station
   * @param rot angular rate of bot CCW in radians
   * @param isFieldRelative if bot should be in field relative control
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean isFieldRelative){

    //field relative or not, forward always away from drive station
    if(isFieldRelative){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getChassisAngle());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,rot);
    }

    
    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);

  }

  /**
   * Set module states for all of the modules. In an array of states in same order as kinimatics
   * @param moduleStates list of states
   */
  public void setModuleStates(SwerveModuleState[] moduleStates){
    //desaturate wheel speeds to under max
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ModuleConstants.MAX_SPEED);

    //assign states to the modules
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }

  /**
  * @return postion (pose) of bot on field
  */  
  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  /**
   * Reset odometry to new position on the field. Uses current heading of the bot but changes field pose
   * @param pose new pose2d
   */
  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(pose, getChassisAngle());
  }


  /**
   * Homes all the swerve modules. 
   * Call resetHomeStatus() before to initialize the process
   */
  public void homeAllModules(){
    frontLeft.home();
    frontRight.home();
    backLeft.home();
    backRight.home();

    //if all modules are done then change variable state to true
    allModuleHomeStatus = frontLeft.homeFinished && frontRight.homeFinished && backLeft.homeFinished && backRight.homeFinished;

  }

  /**
   * Resets all the modules' home status to false
   */
  public void resetHomeStatus(){
    frontLeft.setHomeStatus(false);
    frontRight.setHomeStatus(false);
    backLeft.setHomeStatus(false);
    backRight.setHomeStatus(false);

    allModuleHomeStatus = false;
  }

  /**
   * @return current chassis angle
   */
  public Rotation2d getChassisAngle(){
    return gyro.getRotation2d();
  }

  /**
   * Stops all modules
   */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  /**
   * Resets encoders for all modules
   */
  public void ZeroEncoders() {
    frontLeft.ZeroEncoders();
    frontRight.ZeroEncoders();
    backLeft.ZeroEncoders();
    backRight.ZeroEncoders();
  }

  /**
   * recalibrates the gyroscope. This takes a significant amount of time so make sure bot is stationary
   */
  public void calibrateGyro(){
    gyro.calibrate();
  }

  /**
   * test code for front left module
   * @param angle target angle radians
   */
  public void setSteerAngle(double angle){
    frontLeft.setSteerAngle(angle);
  }




}
