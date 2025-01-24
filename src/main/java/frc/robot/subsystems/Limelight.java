// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  static NetworkTableEntry tx = table.getEntry("tx");
  static NetworkTableEntry ty = table.getEntry("ty");
  static NetworkTableEntry ta = table.getEntry("ta");
  static NetworkTableEntry tv = table.getEntry("tv");

  /** Creates a new Limelight. */
  public Limelight() {
    // table.getEntry("pipeline").setString("April Tag");
    
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); 
    setToAprilTagPipeline();
  }

  public static void updateLimelightValues() {
    getLimelightArea();
    getLimelightY();
    getLimelightx();
    hasValidTarget();
  }

  public void setToAprilTagPipeline() {
    table.getEntry("pipeline").setNumber(0);
  }

  public static void updateLimelightVals() {
    tx.getDouble(0.0);
    ty.getDouble(0.0);
    ta.getDouble(0.0);
  }

  public static double getLimelightx() {
    return tx.getDouble(0.0);
  }

  public static double getLimelightY() {
    return ty.getDouble(0.0);
  }

  public static double getLimelightArea() {
    return ta.getDouble(0.0);
  }

  public static boolean hasValidTarget() {
    return (tv.getDouble(0.0) == 1) ? true : false;
  }

  public static double getTargetDistance() {

    double angleToGoalDegrees = Constants.Limelight.CAMERA_MOUNT_ANGLE_DEG + ty.getDouble(0.0);
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
    double distanceFromLimelightToGoalInches = (Constants.Limelight.GOAL_HEIGHT_INCHES - Constants.Limelight.CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

    double distanceToTarget = distanceFromLimelightToGoalInches - Constants.Limelight.CAMERA_TO_EDGE_OF_ROBOT_INCHES;

    return distanceToTarget;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Limelight Y", NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0));
    SmartDashboard.putNumber("Limelight X", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0));
    SmartDashboard.putNumber("Limelight Area", NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0));
    SmartDashboard.putNumber("Has Target", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0));
    SmartDashboard.putNumber("Target Distance", getTargetDistance());
  }
}
