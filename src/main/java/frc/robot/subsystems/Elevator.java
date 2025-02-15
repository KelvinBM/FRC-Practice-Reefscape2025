// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  // encoder setpoint values for elevator
  private final double kLevel1EncoderPosition = 100;
  private final double kLevel2EncoderPosition = 210;
  private final double kLevel3EncoderPosition = 295;// 295 - 300 // RIGHT_ENCODER
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

  /** Creates a new Elevator. */
  public Elevator() {
    setToCoastMode();
    // setToBrakeMode();
    resetMotorsEncoderPosition();
    elevatorMotorConfigs();
    rightMotorEncoder.setPosition(0);
    leftMotorEncoder.setPosition(0);
  }

  public void elevatorMotorConfigs() {
    elevatorRightMotor.clearFaults();
    elevatorLeftMotor.clearFaults();

    elevatorRightConfig.closedLoop
      .p(0.1)
      .d(0);

    elevatorLeftConfig.closedLoop
      .p(0.1)
      .d(0);

    elevatorRightConfig.inverted(false)
      .follow(elevatorRightMotor);

    elevatorLeftConfig.inverted(true);

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
    SmartDashboard.putNumber("Elevator Right Position", getRightEncoderPosition());
    SmartDashboard.putNumber("Elevator Left Postion", getLeftEncoderPosition());
  }

  public void resetMotorsEncoderPosition() {
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
    elevatorLeftMotor.stopMotor();
  }

  public void raiseElevator(double speed) {
    elevatorLeftMotor.set(speed);
    elevatorRightMotor.set(speed);
  }

  public void lowerElevator(double speed) {
    elevatorLeftMotor.set(-speed);
    elevatorRightMotor.set(-speed);
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////
  ///                                                                                           ///
  /// BOOLEAN COMMANDS                                                                          ///
  ///                                                                                           ///
  ///////////////////////////////////////////////////////////////////////////////////////////////// 
  public boolean hasReachedLevel1() {
    return Math.floor(leftMotorEncoder.getPosition()) == kLevel1EncoderPosition;
  }

  public boolean hasReachedLevel2() {
    return Math.floor(leftMotorEncoder.getPosition()) == kLevel2EncoderPosition;
  }

  public boolean hasReachedLevel3() {
    return Math.floor(leftMotorEncoder.getPosition()) == kLevel3EncoderPosition;
  }

  public boolean hasReachedHumanStation() {
    return Math.floor(leftMotorEncoder.getPosition()) == kHumanStationEncoderPosition;
  }

  public void goToLevel1() {
    if(getRightEncoderPosition() < kLevel1EncoderPosition) {
      rightClosedLoopController.setReference(kLevel1EncoderPosition, ControlType.kPosition);
    } else
      stopAllMotors();
  }

  public void goToLevel2() {
    if(getRightEncoderPosition() < kLevel2EncoderPosition) {
      rightClosedLoopController.setReference(kLevel2EncoderPosition, ControlType.kPosition);
    } else
      stopAllMotors();
  }

  public void goToLevel3() {
    if(getRightEncoderPosition() < kLevel3EncoderPosition) {
      rightClosedLoopController.setReference(kLevel3EncoderPosition, ControlType.kPosition);
    } else
      stopAllMotors();
  }

  public void goToHumanStation() {
    if(getRightEncoderPosition() > kHumanStationEncoderPosition) {
      rightClosedLoopController.setReference(kHumanStationEncoderPosition, ControlType.kPosition);
    } else
      stopAllMotors();
  }

  public void lowerToStartPosition() {
    if(getRightEncoderPosition() > kStartingPosition) {
      rightClosedLoopController.setReference(kStartingPosition, ControlType.kPosition);
    } else
      stopAllMotors();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putElevatorValuesInSmartDashboard();// might not work
  }
}
