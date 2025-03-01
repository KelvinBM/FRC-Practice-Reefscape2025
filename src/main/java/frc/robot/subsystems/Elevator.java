// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  // encoder setpoint values for elevator
  private final double kLevel1EncoderPosition = 100;
  private final double kLevel2EncoderPosition = 210;// 210
  private final double kLevel3EncoderPosition = 300;// 325 // RIGHT_ENCODER
  private final double kHumanStationEncoderPosition = 39;
  private final double kStartingPosition = 5;// actually 0

  // PID constants
  private final double kP = 0.001;

  // motors
  private SparkMax leftMotor = new SparkMax(ElevatorConstants.ELEVATOR_TOP_MOTOR_ID, MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(ElevatorConstants.ELEVATOR_BOTTOM_MOTOR_ID, MotorType.kBrushless);

  // configs
  private SparkMaxConfig rightConfig = new SparkMaxConfig();
  private SparkMaxConfig leftConfig = new SparkMaxConfig();

  // encoders
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private RelativeEncoder leftEncoder = leftMotor.getEncoder();

  // closedloop
  private SparkClosedLoopController rightClosedLoopController =  rightMotor.getClosedLoopController();
  private SparkClosedLoopController leftClosedLoopController =  leftMotor.getClosedLoopController();

  // limit switch
  private DigitalInput elevatorLimitSwitch = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH_PORT);

  /** Creates a new Elevator. */
  public Elevator() {
    setToCoastMode();
    // setToBrakeMode();
    resetMotorsEncoderPositions();
    elevatorMotorConfigs();
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public void elevatorMotorConfigs() {
    rightMotor.clearFaults();
    leftMotor.clearFaults();

    MAXMotionConfig motionConfig = new MAXMotionConfig();

    // SmartMotion Config = similar result as trapezoidal profiling
    motionConfig.maxAcceleration(0.1)
      .maxVelocity(0.6);

    rightConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .maxMotion.apply(motionConfig);// applied through closedLoop's maxMotion config

    leftConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .smartMotion
        .maxAcceleration(kHumanStationEncoderPosition);

    rightConfig.inverted(false)
      .smartCurrentLimit(40)
      .follow(rightMotor);

    leftConfig.inverted(true)
      .smartCurrentLimit(40);

    rightMotor.configure(rightConfig, null, null);
    leftMotor.configure(leftConfig, null, null);
  }

  public void setToBrakeMode() {
    rightConfig.idleMode(IdleMode.kBrake);
    leftConfig.idleMode(IdleMode.kBrake);
  }

  public void setToCoastMode() {
    rightConfig.idleMode(IdleMode.kCoast);
    leftConfig.idleMode(IdleMode.kCoast);
  }

  public double getRightEncoderPosition() {
    return Math.floor(rightEncoder.getPosition());
  }

  public double getLeftEncoderPosition() {
    return Math.floor(leftEncoder.getPosition());
  }

  public void putElevatorValuesInSmartDashboard() {
    SmartDashboard.putBoolean("Elevator Limit Switch", touchingLimitSwitch());
    SmartDashboard.putNumber("Elevator Right Encoder Position", rightEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Left Encoder Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Right Amps", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Left Amps", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Right Amps", rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Left Amps", leftMotor.getOutputCurrent());
  }

  public void resetMotorsEncoderPositions() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  
  /***** PID for elevator method *****/
  /**
   * 
   * @param encoderPosition
   * @param setpoint
   * @return output of elevator motors
   */
  public double pidForElevator(double encoderPosition, double setpoint) {
    double error = setpoint - encoderPosition;
    return kP * error;
  }

  //---------- ELEVATOR METHODS ----------//
  public void stopAllMotors() {
    rightMotor.stopMotor();
  }

  public void raiseElevator(double speed) {
    rightMotor.set(speed);
  }

  public void lowerElevator(double speed) {
    rightMotor.set(-speed);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  ///                                                                                           ///
  /// BOOLEAN COMMANDS                                                                          ///
  ///                                                                                           ///
  ///////////////////////////////////////////////////////////////////////////////////////////////// 
  public boolean touchingLimitSwitch() {
    return !elevatorLimitSwitch.get();
  }

  public boolean hasReachedLevel1() {
    return Math.floor(rightEncoder.getPosition()) == kLevel1EncoderPosition;
  }

  public boolean hasReachedLevel2() {
    return Math.floor(rightEncoder.getPosition()) == kLevel2EncoderPosition;
  }

  public boolean hasReachedLevel3() {
    return Math.floor(rightEncoder.getPosition()) == kLevel3EncoderPosition;
  }

  public boolean hasReachedHumanStation() {
    return Math.floor(rightEncoder.getPosition()) == kHumanStationEncoderPosition;
  }

  public void goToLevel1() {
    rightClosedLoopController.setReference(kLevel1EncoderPosition, ControlType.kPosition);
  }

  public void goToLevel2() {
    rightClosedLoopController.setReference(kLevel2EncoderPosition, ControlType.kPosition);
  }

  public void goToLevel3() {
    rightClosedLoopController.setReference(kLevel3EncoderPosition, ControlType.kPosition);
  }

  public void goToHumanStation() {
    rightClosedLoopController.setReference(kHumanStationEncoderPosition, ControlType.kPosition);
  }

  public void lowerToStartPosition() {
    rightClosedLoopController.setReference(kStartingPosition, ControlType.kPosition);
  }
  
  @Override
  public void periodic() {
    if (touchingLimitSwitch())
      resetMotorsEncoderPositions();

    putElevatorValuesInSmartDashboard();// might not work
  }
}
