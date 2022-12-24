// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PhotonConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {
  
  private SwerveDrivePoseEstimator poseEstimator; 
  private PhotonCamera photonCamera;
  private double previousPipelineTimestamp = 0;
  private Drivetrain drivetrain;
    
  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem(Drivetrain drivetrain, PhotonCamera photonCamera) {
    this.drivetrain = drivetrain;
    this.photonCamera = photonCamera;

    poseEstimator = new SwerveDrivePoseEstimator(
      drivetrain.getChassisAngle(), new Pose2d(), drivetrain.kinematics,
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02,0.02,0.02), 
      new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02), 
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1,0.1,0.1),
      0.02);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var pipelineResult = photonCamera.getLatestResult();
    var resultTimestamp = pipelineResult.getTimestampSeconds();

    
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimestamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      if (target.getPoseAmbiguity() <= .2 && fiducialId == 0) {
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = PhotonConstants.TARGET_POSE.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(PhotonConstants.CAMERA_TO_ROBOT).toPose2d();
        poseEstimator.addVisionMeasurement(visionMeasurement, resultTimestamp);
      }
    }

    poseEstimator.update(drivetrain.getChassisAngle(), drivetrain.getModuleStates());

    SmartDashboard.putString("Robot Pose", getRobotPose().toString());
  }


  public Pose2d getRobotPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(newPose, drivetrain.getChassisAngle());
  }

}
