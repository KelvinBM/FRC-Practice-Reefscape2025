// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralGrabber extends SubsystemBase {
  private SparkMax neoMotor = new SparkMax(8, MotorType.kBrushless);//  might have to change
  private RelativeEncoder encoder = neoMotor.getEncoder();
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  /** Creates a new CoralGrabber. */
  public CoralGrabber() {
    neoConfigs();
  }

  public void neoConfigs() {
    neoMotor.clearFaults();
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.4)
      .i(0)
      .d(0)
      .outputRange(-1, 1)
      .p(0.00001, ClosedLoopSlot.kSlot1)
      .i(0, ClosedLoopSlot.kSlot1)
      .d(0, ClosedLoopSlot.kSlot1);
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }

  public double getEncoderVelocity() {
    return encoder.getVelocity();
  }

  public void stopMotor() {
    neoMotor.set(0);
  }

  // 90 degrees --> figure out
  public boolean setToDegs(double speed, double targetEncoderPosition) {
    while (getEncoderPosition() != targetEncoderPosition) {
      if(getEncoderPosition() < targetEncoderPosition)
        neoMotor.set(speed);
      else if( getEncoderPosition() > targetEncoderPosition)
        neoMotor.set(-speed);
    }
    neoMotor.set(0);
    return true;
  }

  public void stop() {
    neoMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NeoEncoderPosition", getEncoderPosition());
    SmartDashboard.putNumber("NeoEncoderVelocity", getEncoderVelocity());
    SmartDashboard.updateValues();
    // This method will be called once per scheduler run
  }
}
