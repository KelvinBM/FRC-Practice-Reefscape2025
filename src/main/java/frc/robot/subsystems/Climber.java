// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonFX climberAdjuster = new TalonFX(Constants.CLIMBER_ADJUSTER_MOTOR_ID);
  private TalonFX climberRopePuller = new TalonFX(Constants.CLIMBER_ROPE_PULLER_MOTOR_ID);

  /** Creates a new Climber. */
  public Climber() {
    TalonFXConfiguration climberAdjusterConfig = new TalonFXConfiguration();
    TalonFXConfiguration climberRopePullerConfig = new TalonFXConfiguration();

    climberAdjusterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;// might need to invert
    climberRopePullerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;// might need to invert

    climberAdjuster.getConfigurator().apply(climberAdjusterConfig);
    climberRopePuller.getConfigurator().apply(climberAdjusterConfig);

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

  public void climberHookDown(double speed) {
    climberAdjuster.set(-speed);
  }

  public void pullRope(double speed) {
    climberRopePuller.set(speed);
  }

  public void releaseRope(double speed) {
    climberRopePuller.set(-speed);
  }

  public void stopAllMotors() {
    climberAdjuster.stopMotor();
    climberRopePuller.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    uploadAdjusterPosition();
  }
}
