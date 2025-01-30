// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralGrabberConstants;

public class CoralGrabber extends SubsystemBase {
  // motors
  private SparkMax coralAdjustMotor = new SparkMax(CoralGrabberConstants.CORAL_ADJUST_MOTOR_ID, MotorType.kBrushless);//  might have to change
  private SparkMax coralRightMotor = new SparkMax(CoralGrabberConstants.CORAL_MOTOR_RIGHT_ID, MotorType.kBrushless);
  private SparkMax coralLeftMotor = new SparkMax(CoralGrabberConstants.CORAL_MOTOR_LEFT_ID, MotorType.kBrushless);

  // encoders
  private RelativeEncoder coralAdjustEncoder = coralAdjustMotor.getEncoder();
  // private RelativeEncoder coralRightEncoder = coralRightMotor.getEncoder();// may not be needed
  // private RelativeEncoder coralLeftEncoder = coralLeftMotor.getEncoder();// may not be needed

  // configs
  private SparkMaxConfig coralAdjustConfig = new SparkMaxConfig();
  private SparkMaxConfig coralRightConfig = new SparkMaxConfig();
  private SparkMaxConfig coralLeftConfig = new SparkMaxConfig();

  private DigitalInput coralBeamBreaker = new DigitalInput(CoralGrabberConstants.BEAM_BREAKER_PORT);

  private SparkClosedLoopController closedLoopController = coralAdjustMotor.getClosedLoopController();

  /** Creates a new CoralGrabber. */
  public CoralGrabber() {
    coralMotorsConfigs();
  }

  private void coralMotorsConfigs() {
    coralAdjustMotor.clearFaults();
    coralRightMotor.clearFaults();
    coralLeftMotor.clearFaults();

    
    coralAdjustConfig.inverted(false)
      .idleMode(IdleMode.kBrake);// may have to change
    coralRightConfig.inverted(true)
      .idleMode(IdleMode.kBrake);// may have to change to false
    coralLeftConfig.inverted(false)
      .idleMode(IdleMode.kBrake);// may have to change to true

    coralAdjustConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.4)
      .i(0)
      .d(0)
      .outputRange(-1, 1);
      closedLoopController.setReference(30, ControlType.kCurrent);
    coralRightConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0)
      .i(0)
      .d(0)
      .outputRange(-1, 1);
    coralLeftConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0)
      .i(0)
      .d(0)
      .outputRange(-1, 1);
    
    coralAdjustMotor.configure(coralAdjustConfig, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters
    );
    coralRightMotor.configure(coralRightConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );
    coralLeftMotor.configure(coralLeftConfig, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  // public double getEncoderPosition() {
  //   return coralAdjustEncoder.getPosition();
  // }

  // public double getEncoderVelocity() {
  //   return coralAdjustEncoder.getVelocity();
  // }

  // 90 degrees --> figure out
  public Command up(double speed, double targetEncoderPosition) {
    return run(() -> {
      while (coralAdjustEncoder.getPosition() != targetEncoderPosition) {
        if(coralAdjustEncoder.getPosition() < targetEncoderPosition)
          coralAdjustMotor.set(speed);
      }
      coralAdjustMotor.stopMotor();
    });
  }
  // remove targetEncoderPosition after figuring out the encoder value
  public Command down(double speed, double targetEncoderPosition) {
    return run(() -> {
      while (coralAdjustEncoder.getPosition() != targetEncoderPosition) {
        if(coralAdjustEncoder.getPosition() > targetEncoderPosition)
          coralAdjustMotor.set(-speed);
      }
      coralAdjustMotor.stopMotor();
    });
  }

  public Command grabCoral(double speed) {
    return run(() -> {
      while(coralBeamBreaker.get())
      coralLeftMotor.set(speed);
      coralRightMotor.set(speed);
    });
  }

  public Command stopAll() {
    return run(() -> {
      coralAdjustMotor.stopMotor();
      coralRightMotor.stopMotor();
      coralLeftMotor.stopMotor();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CoralGrabber Adjuster Position", coralAdjustEncoder.getPosition());
    SmartDashboard.putBoolean("Has Coral", coralBeamBreaker.get());
  }
}
