// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PhotonSubsystem;

public class TrackCommand extends CommandBase {

  PIDController yawPID = new PIDController(0.1, 0, 0);

  private PhotonSubsystem photonSubsystem;
  private Drivetrain drivetrain;

  /** Creates a new TrackCommand. */
  public TrackCommand(PhotonSubsystem photonSubsystem, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.photonSubsystem = photonSubsystem;
    this.drivetrain = drivetrain;
    yawPID.setTolerance(3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = yawPID.calculate(photonSubsystem.getYaw()) + (Math.signum(photonSubsystem.getYaw()) * -0.3);

    drivetrain.drive(0, 0, output , false);
    SmartDashboard.putNumber("Yaw", photonSubsystem.getYaw());
    SmartDashboard.putNumber("Output", output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
