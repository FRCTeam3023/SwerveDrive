// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.Constants.MMConstants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {

    private WPI_TalonFX driveMotor;
    private CANSparkMax turnMotor;
    private double moduleOffset;
    private DigitalInput hallEffectSensor;
    public int moduleID;
    private RelativeEncoder turnEncoder;
    private SparkMaxPIDController turnPIDController;

    private Gains turnGains = new Gains(.8,0,0,0,0,1);


    private Gains driveGains = new Gains(0.05,0,0,0.046,0,1);

    private boolean homeStatus = false;
    public boolean homeFinished = false;
    private boolean lastState;

    private Timer timer = new Timer();


    /**
     * A single swerve module object
     * @param moduleID Module number
     * @param driveID CAN ID number for the drive Falcon500 
     * @param turnID CAN ID number for the turning CANSparkMax
     * @param moduleOffset radian offset for the zero position in relation to the hall effect sensor
     * @param isInverted set true if the drive motor should be inverted. (Preference depending on set module offset)
     * @param sensorID DIO pin number of the hall effect sensor
     */

    public SwerveModule(int moduleID, int driveID, int turnID, double moduleOffset, boolean isInverted, int sensorID){

        this.moduleOffset = moduleOffset;
        this.moduleID = moduleID;

        hallEffectSensor = new DigitalInput(sensorID);

        //main drive motor
        driveMotor = new WPI_TalonFX(driveID);

        //Reset, basic config
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.set(ControlMode.PercentOutput, 0);
        driveMotor.configClosedloopRamp(0.25);

        driveMotor.setInverted(isInverted);

        //select sensor for main PID loop
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, MMConstants.PRIMARY_PID_LOOP_IDX ,  MMConstants.TIMEOUT_MS);

        //config gains for main PID loop
        driveMotor.config_kP(MMConstants.PRIMARY_PID_LOOP_IDX, driveGains.kP);
        driveMotor.config_kI(MMConstants.PRIMARY_PID_LOOP_IDX, driveGains.kI);
        driveMotor.config_kD(MMConstants.PRIMARY_PID_LOOP_IDX, driveGains.kD);
        driveMotor.config_kF(MMConstants.PRIMARY_PID_LOOP_IDX, driveGains.kF);

        //peak Output
        driveMotor.configPeakOutputForward(driveGains.kPeakOutput, MMConstants.TIMEOUT_MS);
        driveMotor.configPeakOutputReverse(-driveGains.kPeakOutput, MMConstants.TIMEOUT_MS);



        /*-------------------------------------------------------------------------------------------*/



        //main turn motor
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        //reset and create PID
        turnMotor.restoreFactoryDefaults();
        turnMotor.setInverted(true);
        turnPIDController = turnMotor.getPIDController();
        
        //reduce shocking turn speeds, for testing and smoother driving
        turnMotor.setClosedLoopRampRate(0.3);



        //set up encoder on the NEO
        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(2 * Math.PI * (1/ModuleConstants.TURN_GEARING)); //changes rotations to Radians
        turnEncoder.setVelocityConversionFactor(2 * Math.PI * (1/ModuleConstants.TURN_GEARING) / 60 ); //changes rpm to rad/sec


        //set up PID gains for the turning motor
        turnPIDController.setP(turnGains.kP);
        turnPIDController.setI(turnGains.kI);
        turnPIDController.setD(turnGains.kD);
        turnPIDController.setFF(turnGains.kF);
        turnPIDController.setIZone(turnGains.kIzone);
        turnPIDController.setOutputRange(-turnGains.kPeakOutput, turnGains.kPeakOutput);

        turnPIDController.setOutputRange(-0.5, 0.5);

        turnEncoder.setPosition(0);
    }

    

    /**
     * Set the desired state for the module. Drive speeds of 0 will result in no movement of module. 
     * This prevents recentering of module after no joystick input
     * @param desiredState the desired state
     */
    public void setDesiredState(SwerveModuleState desiredState){

        //prevents returning back to 0 state when not moving
        if(Math.abs(desiredState.speedMetersPerSecond) < .001){
            stop();
            return;
        }

        //get current relative encoder pos
        double currentAngleRadians = turnEncoder.getPosition();

        //modulus of current angle, never exeed [0, 2pi)
        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        //optimize the state for the current angle, prevents more than 90 degrees of motion at once
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(currentAngleRadiansMod));

        /*  m/s to units/100ms 
         *  m/s * rev/meters * units/rev * gearing * .1 (to get sec to 100ms)
         */
        double targetVelocity_Units = state.speedMetersPerSecond / Units.inchesToMeters(ModuleConstants.WHEEL_DIA * Math.PI) * Constants.FALCON_UNITS_PER_REV * ModuleConstants.DRIVE_GEARING * 0.1;
        //set to unit speed
        driveMotor.set(ControlMode.Velocity, targetVelocity_Units);

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        double referenceAngleRadians = state.angle.getRadians();
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        //set reference angle to this new adjustment
        turnPIDController.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);

    }


    /**
     * Homes the swerve module, to reset the encoder data
     */
    public void home(){
        //only triggers on rising edge of switch, set new home state and target location
        if(getSwitch() != lastState  && getSwitch()){
            turnEncoder.setPosition(moduleOffset);
            homeStatus = true;
            timer.start();
        }

        //if it is triggered, set to target
        if(homeStatus){
            turnPIDController.setReference(moduleOffset, CANSparkMax.ControlType.kPosition);
        } else {
            //or keep rotating
            turnMotor.set(0.25);
        }

        lastState = getSwitch();

        if(timer.get() > 0.5){
            timer.stop();
            timer.reset();
            homeFinished = true;
        }
    }



    /**
     * @return state of the hall effect sensor
     */
    public boolean getSwitch(){
        return !hallEffectSensor.get();
    }

    /**
     * @return speed of drive motor in meters/second
     */
    public double getSpeed(){
        return driveMotor.getSelectedSensorVelocity() / Constants.FALCON_UNITS_PER_REV * 10 / ModuleConstants.DRIVE_GEARING * Units.inchesToMeters(ModuleConstants.WHEEL_DIA * Math.PI);
    }

    /**
     * Angle of module. This is the unbound version raw from encoder
     * @return angle of module 
     */
    public Rotation2d getAngle(){
        return new Rotation2d(turnEncoder.getPosition());
    }

    /**
     * Set the home state. Called before homing module
     */
    public void setHomeStatus(boolean state){
        homeStatus = state;
        homeFinished = state;
        lastState = getSwitch();
    }

    /**
     * Direct control of motor output percent
     * @param drive drive motor speed
     * @param turn turn motor speed
     */
    public void setMotorSpeeds(double drive, double turn){
        turnMotor.set(turn);
        driveMotor.set(drive);
    }

    /**
     * @return state of module 
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    /**
     * stop module.
     * Sets speeds to 0
     */
    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Resets the encoder position for both motors
     */
    public void ZeroEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turnEncoder.setPosition(0);
    }
    
    /**
     * test code for manual movement of steering angle 
     * @param angle target angle in radians
     */
    public void setSteerAngle(double angle){
        turnPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }
}
