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

  // PID constants
  private final double kP = 0.1;

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
    elevatorMotorConfigs();
    resetMotorsEncoderPosition();
  }

  public void elevatorMotorConfigs() {
    elevatorRightMotor.clearFaults();
    elevatorLeftMotor.clearFaults();

    leftClosedLoopController
      .setReference(30, ControlType.kCurrent);
    
    rightClosedLoopController
      .setReference(30, ControlType.kCurrent);

    elevatorRightConfig.inverted(false)// may have to change
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(35)
      .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)// right pid
      .d(0);

    elevatorLeftConfig.inverted(true)// may have to change
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(35)
      .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)// left pid
      .d(0);

    elevatorRightMotor.configure(elevatorRightConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
    elevatorLeftMotor.configure(elevatorLeftConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
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
    return rightMotorEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return leftMotorEncoder.getPosition();
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
  public void goToFirstLevel_Pid(double setpoint) { // remove param 'setpoint' after figuring it out
    if(leftMotorEncoder.getPosition() < setpoint) {
      double speed = pidForElevator(leftMotorEncoder.getPosition(), setpoint);
      elevatorRightMotor.set(speed);
      elevatorLeftMotor.set(speed);
    } else
      stopAllMotors();
  }

  public void goToStartingPositioin() {
    final double setpoint = 5;
    if(getRightEncoderPosition() > setpoint) {
      double speed = pidForElevator(leftMotorEncoder.getPosition(), setpoint);
      elevatorLeftMotor.set(-speed);
      elevatorRightMotor.set(-speed);
    } else
      stopAllMotors();
  }


  public void stopAllMotors() {
    elevatorRightMotor.stopMotor();
    elevatorLeftMotor.stopMotor();
  }

  /* create methods for elevator */
  // public void goToFirstLevel(double speed) {
  //   if(bottomMotorEncoder.getPosition() < 10)
  //     elevatorMotor_Bottom.set(speed);
  // }

  
  /***** COMMANDS *****/
  //implement encoder position
  public Command elevatorUpCommand(double speed) {
    return run(() -> {
      elevatorLeftMotor.set(speed);
      elevatorRightMotor.set(speed);
    });
  }

  // implement encoder position
  public Command elevatorDownCommand(double speed) {
    return run(() -> {
      elevatorLeftMotor.set(-speed);
      elevatorRightMotor.set(-speed);
    });
  }

  public Command stopAllCommand() {
    return run(() -> {
      elevatorLeftMotor.stopMotor();
      elevatorRightMotor.stopMotor();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putElevatorValuesInSmartDashboard();// might not work
  }
}
