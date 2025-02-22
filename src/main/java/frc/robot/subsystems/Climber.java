// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonFX climberAdjuster = new TalonFX(Constants.CLIMBER_ADJUSTER_MOTOR_ID);
  private TalonFX climberWinch = new TalonFX(Constants.CLIMBER_ROPE_PULLER_MOTOR_ID);

  /** Creates a new Climber. */
  public Climber() {
    TalonFXConfiguration adjusterConfig = new TalonFXConfiguration();
    TalonFXConfiguration winchConfig = new TalonFXConfiguration();

    // climberAdjusterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;// might need to invert
    // climberRopePullerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;// might need to invert
    winchConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    adjusterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // current limits
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.SupplyCurrentLimitEnable  = true;
    currentConfigs.StatorCurrentLimit = 30;
    currentConfigs.SupplyCurrentLimit = 30;

    adjusterConfig.withCurrentLimits(currentConfigs);
    winchConfig.withCurrentLimits(currentConfigs);

    winchConfig.CurrentLimits.StatorCurrentLimit = 30;

    climberAdjuster.getConfigurator().apply(adjusterConfig);
    climberWinch.getConfigurator().apply(winchConfig);

    climberAdjuster.setPosition(0);

    // var slot0Configs = climberAdjusterConfig.Slot0;
    // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
  }

  public void uploadAdjusterPosition() {
    double position = climberAdjuster.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Climber Adjuster Position", position);
  }

  public void climberHookUp(double speed) {
    climberAdjuster.set(speed);
  }

  public void climberHookDown( double adjusterSpeed, double winchSpeed) {
    climberAdjuster.set(-adjusterSpeed);
    climberWinch.set(winchSpeed);
  }

  public void pullRope(double speed) {
    climberWinch.set(speed);
  }

  public void releaseRope(double speed) {
    climberWinch.set(-speed);
  }

  public void climb(double adjusterSpeed, double ropePullerSpeed) {
    climberAdjuster.set(adjusterSpeed);// 0.15
    climberWinch.set(ropePullerSpeed);// 0.25
  }

  /***
   * Stops all of the motors
   */
  public void stopAllMotors() {
    climberAdjuster.stopMotor();
    climberWinch.stopMotor();
  }

  /***
   * Stops the rope adjuster motor
   */
  public void stopWinch() {
    climberAdjuster.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    uploadAdjusterPosition();
  }
}
