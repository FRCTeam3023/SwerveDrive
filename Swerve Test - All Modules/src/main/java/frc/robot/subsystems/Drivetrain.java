// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  /** Creates a new ExampleSubsystem. */

  
  ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  private SwerveModule frontLeft = new SwerveModule(1, 1, 5, ModuleConstants.MODULE1_OFFSET); // Module 1
  private SwerveModule frontRight = new SwerveModule(2, 2, 6, ModuleConstants.MODULE2_OFFSET); // Module 2
  private SwerveModule backLeft = new SwerveModule(3, 3, 7, ModuleConstants.MODULE3_OFFSET); // Module 3
  private SwerveModule backRight = new SwerveModule(4, 4, 8, ModuleConstants.MODULE4_OFFSET); // Module 4

  public final Translation2d frontLeftLocation = new Translation2d(0.5,0.5);
  public final Translation2d frontRightLocation = new Translation2d(0.5, -0.5);
  public final Translation2d backLeftLocation = new Translation2d(-0.5,0.5);
  public final Translation2d backRightLocation = new Translation2d(-0.5, -0.5);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);


  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0));

  public boolean allModuleHomeStatus = false;

  public Drivetrain() {
    gyro.calibrate();
  }

  @Override
  public void periodic() {

    odometer.update(getChassisAngle(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());

    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Module 1 Home State", frontLeft.homeStatus);
    SmartDashboard.putBoolean("Module 2 Home State", frontRight.homeStatus);
    SmartDashboard.putBoolean("Module 3 Home State", backLeft.homeStatus);
    SmartDashboard.putBoolean("Module 4 Home State", backRight.homeStatus);

    SmartDashboard.putNumber("Bot Heading", getChassisAngle().getDegrees());
    SmartDashboard.putString("Bot Position", getPose().getTranslation().toString());

    frontLeft.putMotorData();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Drive the bot using joystick input
   * 
   * @param xSpeed m/s speed positive away from drive station
   * @param ySpeed m/s speed positive left of drive station
   * @param rot angular rate of bot CCW in radians
   */
  public void drive(double xSpeed, double ySpeed, double rot){


    //positive x is forward, y - left, rot - CC rotation
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getChassisAngle());
    
    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);

    SmartDashboard.putNumber("tSpeed", moduleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("tAngle", moduleStates[0].angle.getDegrees());

  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    //desaturate wheel speeds to under max
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ModuleConstants.MAX_SPEED);

    //assign states to the modules
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }

  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometer.resetPosition(pose, getChassisAngle());
  }


  /**
   * Homes all the swerve modules. 
   * Call resetHomeStatus() before to initialize the process
   */
  public void homeAllModules(){
    //if it hasn't finished homing yet then home the module
    if(!frontLeft.homeStatus) frontLeft.home();
    if(!frontRight.homeStatus) frontRight.home();
    if(!backLeft.homeStatus) backLeft.home();
    if(!backRight.homeStatus) backRight.home();
    
    //if all modules are done then change variable state to true
    allModuleHomeStatus = frontLeft.homeStatus; //&& frontRight.homeStatus && backLeft.homeStatus && backRight.homeStatus;

  }

  /**
   * Resets all the modules' home status to false
   */
  public void resetHomeStatus(){
    frontLeft.setHomeStatus(false);
    frontRight.setHomeStatus(false);
    backLeft.setHomeStatus(false);
    backRight.setHomeStatus(false);
  }

  public void drivePercentage(double speed){
    frontLeft.setDriveSpeed(speed);
  }
  public void steerPercentage(double speed){
    frontLeft.setTurnSpeed(speed);
  }

  public Rotation2d getChassisAngle(){
    return gyro.getRotation2d();
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }


}
