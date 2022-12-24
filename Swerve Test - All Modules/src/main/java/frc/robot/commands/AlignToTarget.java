// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AlignToTarget extends CommandBase {
  /** Creates a new AlignToTarget. */
  Drivetrain drivetrain;
  PoseEstimatorSubsystem poseEstimator;

  Pose2d targetPose = new Pose2d(new Translation2d(2, 0), new Rotation2d(Units.degreesToRadians(180)));

  ProfiledPIDController xController = new ProfiledPIDController(0.6, 0, 0, new Constraints(2, 2));
  ProfiledPIDController yController = new ProfiledPIDController(0.6, 0, 0, new Constraints(2, 2));
  ProfiledPIDController thetaController = new ProfiledPIDController(0.5, 0, 0, new Constraints(Constants.MAX_ANGULAR_SPEED, Constants.MAX_ANGULAR_ACCELERATION));



  public AlignToTarget(PoseEstimatorSubsystem poseEstimator, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.poseEstimator = poseEstimator;

    addRequirements(drivetrain);

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentPose = poseEstimator.getRobotPose();

    var xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    var ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    var thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    SmartDashboard.putNumber("X", xSpeed);
    SmartDashboard.putNumber("Y", ySpeed);
    SmartDashboard.putNumber("Theta", thetaSpeed);

    drivetrain.drive(xSpeed, ySpeed, thetaSpeed, true);
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
