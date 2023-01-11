// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Max radians/sec that the bot will rotate at
    public static final double MAX_ANGULAR_SPEED = 4;
    //acceleration of rotation, rad/sec^2
    public static final double MAX_ANGULAR_ACCELERATION = 16;

    
    

    //
    public final class MMConstants{
        public static final int PRIMARY_PID_LOOP_IDX = 0;
        public static final int TIMEOUT_MS = 30;
    }

    public final class ModuleConstants {
        public static final int DRIVE_GEARING = 7; 
        public static final double WHEEL_DIA = 4; //in inches
        public static final double TURN_GEARING = 2.89 * 2.89 * 6;
        public static final double MAX_SPEED = 2; 

        //module offsets for the zero position for each wheel, in this order
        //Front Left - Front Right - Back Left - Back Right
        public static final double MODULE1_OFFSET = -3 * Math.PI/4;
        public static final double MODULE2_OFFSET = 3 * Math.PI/4;
        public static final double MODULE3_OFFSET = 3 * Math.PI/4;
        public static final double MODULE4_OFFSET = -3 * Math.PI/4;



    }

    public static final double FALCON_UNITS_PER_REV = 2048;

    public final class AutoConstants {
        public static final double MAX_ACCELERATION = 1;
        public static final double MAX_SPEED = 2;

        public static final double X_CONTROLLER = 2;
        public static final double Y_CONTROLLER = 2;
        public static final double THETA_CONTROLLER = 2.5;
        public final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED, 
                MAX_ANGULAR_ACCELERATION);
    }

    public static class PhotonConstants {
        public final static double CAM_PITCH = 15; //degrees

        public final static Transform3d CAMERA_TO_ROBOT = 
            new Transform3d(
                new Translation3d(-Units.inchesToMeters(21/2), 0, -Units.inchesToMeters(6)), 
                new Rotation3d(Units.degreesToRadians(CAM_PITCH),0,0)
            );

        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

        public static final Pose3d TARGET_POSE = new Pose3d(0, 0, Units.inchesToMeters(23.5), new Rotation3d());
    }

}
