// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.HomeCommand;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Timer;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  private final Joystick mainJoystick = new Joystick(1);
  
  private final JoystickDrive joystickDrive = new JoystickDrive(drivetrain, mainJoystick);

  String trajectoryJSON = "paths/Forward-1.wpilib.json";
  Trajectory trajectory = new Trajectory();

  Timer timer = new Timer(); // for auto



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain.setDefaultCommand(joystickDrive);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
     * Current Button Binding layout:
     * 
     * 
     * 1: Home all modules
     * 7: Recalibrate Gyro
     * 10: Module 1 - 0 turns
     * 11: Module 1 - 4 turns
     * 12: Module 1 - 8 turns
     * 
     * 
     * 
     */


    new JoystickButton(mainJoystick, 1).whenHeld(new HomeCommand(drivetrain));

    //zero Gyro angle, counter drift during testing. Hopefully get a better gyro soon  (Will make a loop overrun warning)
    new JoystickButton(mainJoystick, 7).whenPressed(new InstantCommand(() -> drivetrain.calibrateGyro()));

    //test of module steering accuracy, turn 0,4,8 rotations
    new JoystickButton(mainJoystick, 10).whenHeld(
      new RunCommand(() -> drivetrain.setSteerAngle(0), drivetrain)
    );
    new JoystickButton(mainJoystick, 11).whenHeld(
      new RunCommand(() -> drivetrain.setSteerAngle(8 * Math.PI), drivetrain)
    );
    new JoystickButton(mainJoystick, 12).whenHeld(
      new RunCommand(() -> drivetrain.setSteerAngle(16 * Math.PI), drivetrain)
    );
    
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_SPEED, AutoConstants.MAX_ACCELERATION)
                                        .setKinematics(drivetrain.kinematics);

    TrajectoryConfig config2 = new TrajectoryConfig(AutoConstants.MAX_SPEED, AutoConstants.MAX_ACCELERATION)
                                        .setKinematics(drivetrain.kinematics)
                                        .setReversed(true);
                                        

    //creation of both trajectories 
    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)),
      List.of(
        new Translation2d(1,0),
        new Translation2d(1,-1)
      ),
      new Pose2d(2,-1,Rotation2d.fromDegrees(0)), 
      config
    );

    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2,-1,Rotation2d.fromDegrees(0)),
      List.of(
        new Translation2d(1, 0)
      ),
      new Pose2d(0,0, Rotation2d.fromDegrees(0)), 
      config2
    );

    //combining the two trajectories 
    Trajectory trajectory3 = trajectory1.concatenate(trajectory2);

    //Pid controllers to correct for error in positioning during autonomous x,y,theta
    PIDController xController = new PIDController(AutoConstants.X_CONTROLLER, 0, 0);
    PIDController yController = new PIDController(AutoConstants.Y_CONTROLLER, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.THETA_CONTROLLER, 0, 0,
          new TrapezoidProfile.Constraints(Constants.MAX_ANGULAR_SPEED, Constants.MAX_ANGULAR_ACCELERATION) );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //new swerve controller command. Uses the custom angle setpoint
    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
      trajectory3, 
      drivetrain::getPose, 
      drivetrain.kinematics, 
      xController, 
      yController, 
      thetaController, 
      () -> getAngle(timer.get(), trajectory1.getTotalTimeSeconds(), trajectory2.getTotalTimeSeconds(), trajectory.getTotalTimeSeconds()),
      drivetrain::setModuleStates, 
      drivetrain);

      timer.reset();

    return new SequentialCommandGroup(
      new HomeCommand(drivetrain),
      new InstantCommand(() -> drivetrain.resetOdometry(trajectory1.getInitialPose())),
      new InstantCommand(timer::start),
      swerveControllerCommand1,
      new InstantCommand(timer::stop),
      new InstantCommand(() -> drivetrain.stopModules())
    );
  }

  //returns custom angle setpoint for autonomous
  private Rotation2d getAngle(double currentTime, double time1, double time2, double totalTime){
    //have some function in relation to time
    Rotation2d setpoint;
    if(currentTime < time1 && currentTime >= 0.5){
      setpoint = Rotation2d.fromDegrees(180);
    } else if(currentTime >= time1 && currentTime <= totalTime){
      setpoint = Rotation2d.fromDegrees(0);
    } else {
      setpoint = Rotation2d.fromDegrees(0);
    }


    return setpoint;
  }
}
