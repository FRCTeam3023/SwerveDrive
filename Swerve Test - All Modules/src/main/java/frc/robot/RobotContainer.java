// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HomeCommand;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.Drivetrain;


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

  // This will load the file "Simple Path.path" and generate it with a max velocity of 2 m/s and a max acceleration of 1 m/s^2
  // for every path in the group
  ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Simple Path", new PathConstraints(3, 1.5));

  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    drivetrain::getPose, // Pose2d supplier
    drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    drivetrain.kinematics, // SwerveDriveKinematics
    new PIDConstants(4, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(4, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
  );


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    PathPlannerServer.startServer(5811);
    drivetrain.setDefaultCommand(joystickDrive);

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

      Command fullAuto = autoBuilder.fullAuto(pathGroup);

    return new SequentialCommandGroup(
      new HomeCommand(drivetrain),
      fullAuto
    );
  }

  
}
