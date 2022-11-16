// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final Joystick joystick;

  private double xSpeed;
  private double ySpeed;
  private double rot;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickDrive(Drivetrain drivetrain, Joystick joystick) {
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //take joystick inputs scaled to the max speed of the bot
    //desaturation will lead to never exeeding max on diagonals 
    xSpeed = joystick.getY() * ModuleConstants.MAX_SPEED;
    ySpeed = joystick.getX() * ModuleConstants.MAX_SPEED;
    rot = joystick.getTwist() * Constants.MAX_ANGULAR_SPEED;

    if(Math.abs(xSpeed) < .15) xSpeed = 0;
    if(Math.abs(ySpeed) < .15) ySpeed = 0;
    if(Math.abs(rot) < 0.25) rot = 0;


    drivetrain.drive(xSpeed, ySpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
