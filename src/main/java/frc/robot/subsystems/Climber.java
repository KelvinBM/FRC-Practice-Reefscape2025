// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private SparkMax climberAdjuster = new SparkMax(Constants.CLIMBER_ADJUSTER_MOTOR_ID, MotorType.kBrushless);
  private SparkMax climberRopePuller = new SparkMax(Constants.CLIMBER_ROPE_PULLER_ID, MotorType.kBrushless);
  private SparkMaxConfig climberConfig = new SparkMaxConfig();
  private RelativeEncoder climberEncoder = climberAdjuster.getEncoder();

  /** Creates a new Climber. */
  public Climber() {
    climberAdjuster.clearFaults();

    climberConfig.closedLoop.p(0.1); // pid

    climberAdjuster.configure(
      climberConfig.inverted(false),
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    climberConfig.closedLoop
      .p(0.1)
      .d(0);

    resetEncoderPosition();
  }

  public void adjusterForward(double speed){
    climberAdjuster.set(speed); // may have to make negative
  }

  public void resetEncoderPosition() {
    climberEncoder.setPosition(0);
  }

  public void backward(double speed) {
    climberAdjuster.set(-speed); // may have to make positive
  }

  public void adjustSpeed(double adjustValue) {
    Constants.ClimberInitialSpeed += adjustValue;
  }

  public void climbWithRotatingLimit(double speed) {
    if(climberAdjuster.getAbsoluteEncoder().getPosition() < 100) {
      adjusterForward(speed);
    }
  }

  public void lowerRobotWithLimit(double speed) {
    if(climberAdjuster.getAbsoluteEncoder().getPosition() > 6) {
      adjusterForward(speed);
    }
  }

  public void stopAdjuster() {
    climberAdjuster.stopMotor();
  }

  public void stopRopePuller() {
    climberRopePuller.stopMotor();
  }

  public void stopAllMotors() {
    climberAdjuster.stopMotor();
    climberRopePuller.stopMotor();
  }

  /***** COMMANDS *****/
  public Command adjusterForwardCommand(double speed) {
    return run(() -> climberAdjuster.set(speed));
  }

  public Command adjusterBackwardCommand(double speed) {
    return run(() -> climberAdjuster.set(-speed));
  }

  public Command climberRopeTightenCommand(double speed) {
    return run(() -> climberRopePuller.set(speed));
  }

  public Command climberRopeReleaseCommand(double speed) {
    return run(() -> climberRopePuller.set(-speed));
  }

  public Command climberStopAllCommand() {
    return run(() -> {
      climberAdjuster.stopMotor();
      climberRopePuller.stopMotor();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
