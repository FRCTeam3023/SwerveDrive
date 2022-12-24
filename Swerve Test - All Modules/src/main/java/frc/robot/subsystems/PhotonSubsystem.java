// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.annotation.Target;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

public class PhotonSubsystem extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("visionCamera");

  public Pose2d robotPose;


  /** Creates a new PhotonSubsystem. */
  public PhotonSubsystem() {}

  @Override
  public void periodic() {
    var result = camera.getLatestResult();

    if(result.hasTargets()){
      var target = result.getBestTarget();

      var cameraToTarget = target.getBestCameraToTarget();
      var targetTocamera = cameraToTarget.inverse();

      var cameraPose = PhotonConstants.TARGET_POSE.transformBy(targetTocamera);
      robotPose = cameraPose.transformBy(PhotonConstants.CAMERA_TO_ROBOT).toPose2d();
      SmartDashboard.putString("Vision Pose", robotPose.toString());
    }

  }


  public double getYaw(){
    var result = camera.getLatestResult();

    if(result.hasTargets()){
      var target = result.getBestTarget();

      SmartDashboard.putNumber("Yaw Source", target.getYaw());
      return target.getYaw();
    } else {
      return 0;
    }
  }

  public Pose2d getVisionPose(){
    return robotPose;
  }
}
