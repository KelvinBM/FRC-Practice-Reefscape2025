// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
  // private SparkMax coralAdjustMotor = new SparkMax(CoralGrabberConstants.CORAL_ADJUST_MOTOR_ID, MotorType.kBrushless);// might have to change
  private SparkMax coralRightMotor = new SparkMax(CoralGrabberConstants.CORAL_MOTOR_RIGHT_ID, MotorType.kBrushless);
  private SparkMax coralLeftMotor = new SparkMax(CoralGrabberConstants.CORAL_MOTOR_LEFT_ID, MotorType.kBrushless);

  // encoders
  // private RelativeEncoder coralAdjustEncoder = coralAdjustMotor.getEncoder();
  private RelativeEncoder coralRightEncoder = coralRightMotor.getEncoder();// may not be needed
  private RelativeEncoder coralLeftEncoder = coralLeftMotor.getEncoder();// may not be needed

  // configs
  // private SparkMaxConfig coralAdjustConfig = new SparkMaxConfig();
  private SparkMaxConfig coralRightConfig = new SparkMaxConfig();
  private SparkMaxConfig coralLeftConfig = new SparkMaxConfig();

  // other sensors
  private DigitalInput coralBeamBreaker = new DigitalInput(CoralGrabberConstants.BEAM_BREAKER_PORT);
  private static DigitalInput coralLimitSwitch_Up = new DigitalInput(CoralGrabberConstants.LIMIT_SWITCH_UP_PORT);
  private static DigitalInput coralLimitSwitch_Down = new DigitalInput(CoralGrabberConstants.LIMIT_SWITCH_DOWN_PORT);
  private static DigitalInput hasCoralLimitSwitch = new DigitalInput(CoralGrabberConstants.LIMIT_SWITCH_CORAL_PORT);

  // private SparkClosedLoopController adjusterClosedLoopController = coralAdjustMotor.getClosedLoopController();
  private SparkClosedLoopController rightClosedLoopController = coralRightMotor.getClosedLoopController();
  private SparkClosedLoopController leftClosedLoopController = coralLeftMotor.getClosedLoopController();



  /** Creates a new CoralGrabber. */
  public CoralGrabber() {
    coralMotorsConfigs();
  }

  private void coralMotorsConfigs() {
    // coralAdjustMotor.clearFaults();
    coralRightMotor.clearFaults();
    coralLeftMotor.clearFaults();

    // coralAdjustConfig.inverted(false)// may have to change
    //     .idleMode(IdleMode.kBrake)
    //     .smartCurrentLimit(25);
    coralRightConfig.inverted(true)// may have to change to false
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(25);
    coralLeftConfig.inverted(false)// may have to change to true
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(25);
          
    // adjusterClosedLoopController.setReference(20, ControlType.kCurrent);
    rightClosedLoopController.setReference(20, ControlType.kCurrent);
    leftClosedLoopController.setReference(20, ControlType.kCurrent);

    // coralAdjustConfig.closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .p(0.1)
    //     .i(0)
    //     .d(0)
    //     .outputRange(-1, 1);
    coralRightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    coralLeftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);

    // coralAdjustMotor.configure(coralAdjustConfig,
    //     ResetMode.kNoResetSafeParameters,
    //     PersistMode.kPersistParameters);
    coralRightMotor.configure(coralRightConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
    coralLeftMotor.configure(coralLeftConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // public double getEncoderPosition() {
  // return coralAdjustEncoder.getPosition();
  // }

  // public double getEncoderVelocity() {
  // return coralAdjustEncoder.getVelocity();
  // }

  public void setMotorsToCoast() {
    // coralAdjustConfig.idleMode(IdleMode.kCoast);
    coralRightConfig.idleMode(IdleMode.kCoast);
    coralLeftConfig.idleMode(IdleMode.kCoast);

    // coralAdjustMotor.configure(coralAdjustConfig,
    //     null,
    //     null);
    coralRightMotor.configure(coralRightConfig,
        null,
        null);
    coralLeftMotor.configure(coralLeftConfig,
        null,
        null);
  }

  public void setMotorsToBrake() {
    // coralAdjustConfig.idleMode(IdleMode.kBrake);
    coralRightConfig.idleMode(IdleMode.kBrake);
    coralLeftConfig.idleMode(IdleMode.kBrake);

    // coralAdjustMotor.configure(coralAdjustConfig,
    //     null,
    //     null);
    coralRightMotor.configure(coralRightConfig,
        null,
        null);
    coralLeftMotor.configure(coralLeftConfig,
        null,
        null);
  }

  // public void resetAdjustEncoderPosition() {
  //   coralAdjustEncoder.setPosition(0);
  // }

  public void resetRightEncoderPosition() {
    coralRightEncoder.setPosition(0);
  }

  public void resetLeftEncoderPosition() {
    coralLeftEncoder.setPosition(0);
  }

  // sensor boolean methods
  public static boolean getCoralLimitSwitch_Up() {
    return !coralLimitSwitch_Up.get();
  }

  public static boolean getCoralLimitSwitch_Down() {
    return !coralLimitSwitch_Down.get();
  }

  public static boolean hasCoral() {
    return !hasCoralLimitSwitch.get(); // might have to modify
  }

  // public void up(double speed) {
  //   if (!coralLimitSwitch_Up.get())
  //     coralAdjustMotor.set(speed);
  //   else
  //     coralAdjustMotor.stopMotor();
  // }

  // public void down(double speed) {
  //   if (!coralLimitSwitch_Down.get())
  //     coralAdjustMotor.set(speed);
  //   else
  //     coralAdjustMotor.stopMotor();
  // }

  // 90 degrees --> figure out
  // public boolean upWithEncoder(double speed, double targetEncoderPosition) {
  //   if (coralAdjustEncoder.getPosition() >= targetEncoderPosition)
  //     return true;

  //   coralAdjustMotor.set(speed);
  //   return false;
  // }

  // remove targetEncoderPosition after figuring out the encoder value
  // public boolean downWithEncoder(double speed, double targetEncoderPosition) {
  //   if (coralAdjustEncoder.getPosition() < targetEncoderPosition)
  //     return true;

  //   coralAdjustMotor.set(-speed);
  //   return false;
  // }

  public void grabCoral(double speed) {
    if (!coralBeamBreaker.get())
      coralLeftMotor.set(speed);
      coralRightMotor.set(speed);
  }

  public void releaseCoral(double speed) {
    coralLeftMotor.set(-speed);
    coralRightMotor.set(-speed);
  }

  // public void stopAdjuster() {
  //   coralAdjustMotor.stopMotor();
  // }

  public void stopRightMotor() {
    coralRightMotor.stopMotor();
  }

  public void stopLeftMotor() {
    coralLeftMotor.stopMotor();
  }

  public void stopAllMotors() {
    // coralAdjustMotor.stopMotor();
    coralRightMotor.stopMotor();
    coralLeftMotor.stopMotor();
  }

  /***** COMMANDS *****/
  // implement limit switch
  public Command grabCoralCommand(double speed) {
    return run(() -> {
      coralLeftMotor.set(speed);
      coralRightMotor.set(speed);
    });
  }

  public Command releaseCoralCommand(double speed) {
    return run(() -> {
      coralLeftMotor.set(-speed);
      coralRightMotor.set(-speed);
    });
  }

  public Command stopAllMotorsCommand() {
    return run(() -> {
      coralLeftMotor.stopMotor();
      coralRightMotor.stopMotor();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("CoralGrabber Adjuster Position", coralAdjustEncoder.getPosition());
    SmartDashboard.putBoolean("Has Coral", coralBeamBreaker.get());
  }
}
