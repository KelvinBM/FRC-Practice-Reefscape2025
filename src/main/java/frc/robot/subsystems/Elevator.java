// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
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
  private SparkMax elevatorLeftMotor = new SparkMax(ElevatorConstants.ELEVATOR_TOP_MOTOR_ID, MotorType.kBrushless);
  private SparkMax elevatorRightMotor = new SparkMax(ElevatorConstants.ELEVATOR_BOTTOM_MOTOR_ID, MotorType.kBrushless);

  // configs
  private SparkMaxConfig elevatorRightConfig = new SparkMaxConfig();
  private SparkMaxConfig elevatorLeftConfig = new SparkMaxConfig();

  // encoders
  private RelativeEncoder rightMotorEncoder = elevatorRightMotor.getEncoder();
  private RelativeEncoder leftMotorEncoder = elevatorLeftMotor.getEncoder();

  // closedloop
  private SparkClosedLoopController rightClosedLoopController =  elevatorRightMotor.getClosedLoopController();
  private SparkClosedLoopController leftClosedLoopController =  elevatorLeftMotor.getClosedLoopController();


  // limit switch
  private DigitalInput elevatorLimitSwitch = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH_PORT);

  /** Creates a new Elevator. */
  public Elevator() {
    setToCoastMode();
    // setToBrakeMode();
    resetMotorsEncoderPositions();
    elevatorMotorConfigs();
    rightMotorEncoder.setPosition(0);
    leftMotorEncoder.setPosition(0);
  }

  public void elevatorMotorConfigs() {
    elevatorRightMotor.clearFaults();
    elevatorLeftMotor.clearFaults();

    elevatorRightConfig.closedLoop
      .p(0.1);

    elevatorLeftConfig.closedLoop
      .p(0.1);

    elevatorRightConfig.inverted(false)
      .smartCurrentLimit(40)
      .follow(elevatorRightMotor);

    elevatorLeftConfig.inverted(true)
      .smartCurrentLimit(40);

    elevatorRightMotor.configure(elevatorRightConfig, null, null);
    elevatorLeftMotor.configure(elevatorLeftConfig, null, null);
  }

  public void setToBrakeMode() {
    elevatorRightConfig.idleMode(IdleMode.kBrake);
    elevatorLeftConfig.idleMode(IdleMode.kBrake);
  }

  public void setToCoastMode() {
    elevatorRightConfig.idleMode(IdleMode.kCoast);
    elevatorLeftConfig.idleMode(IdleMode.kCoast);
  }

  public double getRightEncoderPosition() {
    return Math.floor(rightMotorEncoder.getPosition());
  }

  public double getLeftEncoderPosition() {
    return Math.floor(leftMotorEncoder.getPosition());
  }

  public void putElevatorValuesInSmartDashboard() {
    SmartDashboard.putBoolean("Elevator Limit Switch", touchingLimitSwitch());
    SmartDashboard.putNumber("Elevator Right Encoder Position", rightMotorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Left Encoder Position", leftMotorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Right Amps", rightMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Left Amps", leftMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Right Amps", elevatorRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Left Amps", elevatorLeftMotor.getOutputCurrent());
  }

  public void resetMotorsEncoderPositions() {
    leftMotorEncoder.setPosition(0);
    rightMotorEncoder.setPosition(0);
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
    elevatorRightMotor.stopMotor();
  }

  public void raiseElevator(double speed) {
    elevatorRightMotor.set(speed);
  }

  public void lowerElevator(double speed) {
    elevatorRightMotor.set(-speed);
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
    return Math.floor(rightMotorEncoder.getPosition()) == kLevel1EncoderPosition;
  }

  public boolean hasReachedLevel2() {
    return Math.floor(rightMotorEncoder.getPosition()) == kLevel2EncoderPosition;
  }

  public boolean hasReachedLevel3() {
    return Math.floor(rightMotorEncoder.getPosition()) == kLevel3EncoderPosition;
  }

  public boolean hasReachedHumanStation() {
    return Math.floor(rightMotorEncoder.getPosition()) == kHumanStationEncoderPosition;
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
