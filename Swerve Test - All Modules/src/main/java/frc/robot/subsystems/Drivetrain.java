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

  private SwerveModule frontLeft = new SwerveModule(1, 1, 5, ModuleConstants.MODULE1_OFFSET, true, 0); // Module 1
  private SwerveModule frontRight = new SwerveModule(2, 2, 6, ModuleConstants.MODULE2_OFFSET, false, 1); // Module 2
  private SwerveModule backLeft = new SwerveModule(3, 3, 7, ModuleConstants.MODULE3_OFFSET, true, 2); // Module 3
  private SwerveModule backRight = new SwerveModule(4, 4, 8, ModuleConstants.MODULE4_OFFSET, false, 3); // Module 4

  public final Translation2d frontLeftLocation = new Translation2d(0.175,0.175);
  public final Translation2d frontRightLocation = new Translation2d(0.175, -0.175);
  public final Translation2d backLeftLocation = new Translation2d(-0.175,0.175);
  public final Translation2d backRightLocation = new Translation2d(-0.175, -0.175);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);


  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(kinematics, new Rotation2d(0));

  public boolean allModuleHomeStatus = false;

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

    SmartDashboard.putString("Module1 Angle", frontLeft.getAngle().toString());
    SmartDashboard.putString("Module1 Setpoint", frontLeft.getState().angle.toString());

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

  public void ZeroEncoders() {
    frontLeft.ZeroEncoders();
    frontRight.ZeroEncoders();
    backLeft.ZeroEncoders();
    backRight.ZeroEncoders();
  }

  public void calibrateGyro(){
    gyro.calibrate();
  }

  public void setSteerAngle(double angle){
    frontLeft.setSteerAngle(angle);
  }




}
