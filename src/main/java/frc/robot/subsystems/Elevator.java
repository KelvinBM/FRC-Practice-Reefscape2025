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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  // PID constants
  private final double kP = 0.1;

  // motors
  private SparkMax elevatorMotor_Left = new SparkMax(ElevatorConstants.ELEVATOR_TOP_MOTOR_ID, MotorType.kBrushless);
  private SparkMax elevatorMotor_Right = new SparkMax(ElevatorConstants.ELEVATOR_BOTTOM_MOTOR_ID, MotorType.kBrushless);

  // configs
  private SparkMaxConfig elevatorTopConfig = new SparkMaxConfig();
  private SparkMaxConfig elevatorBottomConfig = new SparkMaxConfig();

  // encoders
  private RelativeEncoder rightMotorEncoder = elevatorMotor_Right.getEncoder();
  private RelativeEncoder leftMotorEncoder = elevatorMotor_Left.getEncoder();

  // closedloop
  private SparkClosedLoopController topClosedLoopController =  elevatorMotor_Right.getClosedLoopController();
  private SparkClosedLoopController bottomClosedLoopController =  elevatorMotor_Left.getClosedLoopController();

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorConfigs();
    resetMotorsEncoderPosition();
  }

  public void elevatorMotorConfigs() {
    elevatorMotor_Right.clearFaults();
    elevatorMotor_Left.clearFaults();

    topClosedLoopController.setReference(30, ControlType.kCurrent);
    bottomClosedLoopController.setReference(30, ControlType.kCurrent);

    elevatorTopConfig.inverted(false)// may have to change
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(35);
    elevatorBottomConfig.inverted(true)// may have to change
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(35);

    elevatorMotor_Right.configure(elevatorTopConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
    elevatorMotor_Left.configure(elevatorBottomConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  public void setToBrakeMode() {
    elevatorTopConfig.idleMode(IdleMode.kBrake);
    elevatorBottomConfig.idleMode(IdleMode.kBrake);
     
    elevatorMotor_Right.configure(elevatorTopConfig, null, null);
    elevatorMotor_Left.configure(elevatorBottomConfig, null, null);
  }

  public void setToCoastMode() {
    elevatorTopConfig.idleMode(IdleMode.kCoast);
    elevatorBottomConfig.idleMode(IdleMode.kCoast);
     
    elevatorMotor_Right.configure(elevatorTopConfig, null, null);
    elevatorMotor_Left.configure(elevatorBottomConfig, null, null);
  }

  public double getRightEncoderPosition() {
    return rightMotorEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return leftMotorEncoder.getPosition();
  }

  public void putElevatorValuesInSmartDashboard() {
    SmartDashboard.putNumber("Elevator Right Position Value", getRightEncoderPosition());
    SmartDashboard.putNumber("Elevator Left Postion Value", getLeftEncoderPosition());
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
  public void goToFirstLevel(double setpoint) { // remove param 'setpoint' after figuring it out
    if(leftMotorEncoder.getPosition() < setpoint) {
      double speed = pidForElevator(leftMotorEncoder.getPosition(), setpoint);
      elevatorMotor_Right.set(speed);
      elevatorMotor_Left.set(speed);
    } else
      stopAllMotors();
  }

  public void goToStartingPositioin() {
    final double setpoint = 5;
    if(getRightEncoderPosition() > setpoint) {
      double speed = pidForElevator(leftMotorEncoder.getPosition(), setpoint);
      elevatorMotor_Left.set(-speed);
      elevatorMotor_Right.set(-speed);
    } else
      stopAllMotors();
  }


  public void stopAllMotors() {
    elevatorMotor_Right.stopMotor();
    elevatorMotor_Left.stopMotor();
  }

  /* create methods for elevator */
  // public void goToFirstLevel(double speed) {
  //   if(bottomMotorEncoder.getPosition() < 10)
  //     elevatorMotor_Bottom.set(speed);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putElevatorValuesInSmartDashboard();// might not work
  }
}
