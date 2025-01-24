// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private SparkMax climber = new SparkMax(Constants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig climberConfig = new SparkMaxConfig();

  /** Creates a new Climber. */
  public Climber() {
    climber.clearFaults();

    climber.configure(
      climberConfig.inverted(false),
        null,
      null
    );
  }

  public void forward(double speed){
    climber.set(speed); // may have to make negative
  }

  public void backward(double speed) {
    climber.set(-speed); // may have to make positive
  }

  public void adjustSpeed(double adjustValue) {
    Constants.ClimberInitialSpeed += adjustValue;
  }

  public void stop(){
    climber.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
