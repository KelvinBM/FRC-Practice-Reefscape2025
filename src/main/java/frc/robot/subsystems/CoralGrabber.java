// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralGrabberConstants;

public class CoralGrabber extends SubsystemBase {
  // motors
  // private SparkMax coralAdjustMotor = new SparkMax(CoralGrabberConstants.CORAL_ADJUST_MOTOR_ID, MotorType.kBrushless);// might have to change
  private SparkMax coralRightMotor = new SparkMax(CoralGrabberConstants.CORAL_MOTOR_RIGHT_ID, MotorType.kBrushless);
  private SparkMax coralLeftMotor = new SparkMax(CoralGrabberConstants.CORAL_MOTOR_LEFT_ID, MotorType.kBrushless);


  // configs
  // private SparkMaxConfig coralAdjustConfig = new SparkMaxConfig();
  private SparkMaxConfig coralRightConfig = new SparkMaxConfig();
  private SparkMaxConfig coralLeftConfig = new SparkMaxConfig();

  // other sensors
  private DigitalInput coralBeamBreaker = new DigitalInput(CoralGrabberConstants.BEAM_BREAKER_PORT);


  /** Creates a new CoralGrabber. */
  public CoralGrabber() {
    coralMotorsConfigs();
  }

  private void coralMotorsConfigs() {
    coralRightMotor.clearFaults();
    coralLeftMotor.clearFaults();

    coralRightConfig.inverted(true)// may have to change to false
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30);
 
    coralLeftConfig.inverted(false)// may have to change to true
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30)
        .follow(coralRightMotor);

        
    coralRightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    coralLeftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);

    // coralAdjustMotor.configure(coralAdjustConfig,
    // ResetMode.kNoResetSafeParameters,
    // PersistMode.kPersistParameters);
    coralRightMotor.configure(coralRightConfig, null, null);
    coralLeftMotor.configure(coralLeftConfig, null, null);
  }


  // sensor boolean methods
  public boolean hasCoral() {
    return !coralBeamBreaker.get();
  }

  public void grabCoral(double speed) {
    if (!hasCoral()) {
      coralRightMotor.set(speed);
    }
    if (hasCoral()) {
      stopMotors();
    }
  }

  public void releaseCoral(double speed) {
    coralRightMotor.set(speed);
  }

  public void returnCoral(double speed) {
    coralRightMotor.set(-speed);
  }

  public void stopMotors() {
    coralRightMotor.stopMotor();
    coralLeftMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("CoralGrabber Adjuster Position", coralAdjustEncoder.getPosition());
    SmartDashboard.putBoolean("Has Coral", hasCoral());
  }
}
