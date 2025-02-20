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
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /* Can't access limelight as table object */
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  static NetworkTableEntry tx = table.getEntry("tx");
  static NetworkTableEntry ty = table.getEntry("ty");
  static NetworkTableEntry ta = table.getEntry("ta");
  static NetworkTableEntry tv = table.getEntry("tv");

  /** Creates a new Limelight. */
  public Limelight() {
    setLimelightPipeline(1);
  }

  public void setLimelightPipeline(double pipelineNumber) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(pipelineNumber);
  }

  public static double getTableValue(String tablentry) {
    switch(tablentry) {
      case "tx":
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
      case "ta":
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
      case "ty":
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
      case "tv":
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
      case "pipeline":
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(1);
      default:
        return 0;
    }
  }

  public static boolean hasValidTarget() {
    return (getTableValue("tv") == 1) ? true : false;
  }



  public static double getTargetDistance() {

    double angleToGoalDegrees = Constants.LimelightConstants.CAMERA_MOUNT_ANGLE_DEG + getTableValue("ty");
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
    double distanceFromLimelightToGoalInches = (Constants.LimelightConstants.GOAL_HEIGHT_INCHES - Constants.LimelightConstants.CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

    double distanceToTarget = distanceFromLimelightToGoalInches - Constants.LimelightConstants.CAMERA_TO_EDGE_OF_ROBOT_INCHES;

    return distanceToTarget;
  }

  public static double getTargetDistanceUsingLimelightHelper() {

    double angleToGoalDegrees = Constants.LimelightConstants.CAMERA_MOUNT_ANGLE_DEG + LimelightHelpers.getLimelightNTDouble("limelight", "ty");
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
    double distanceFromLimelightToGoalInches = (Constants.LimelightConstants.GOAL_HEIGHT_INCHES - Constants.LimelightConstants.CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

    double distanceToTarget = distanceFromLimelightToGoalInches - Constants.LimelightConstants.CAMERA_TO_EDGE_OF_ROBOT_INCHES;

    return distanceToTarget;
  }

  public static void putLimelightValuesInDashboard() {
    SmartDashboard.putNumber("Limelight Y", getTableValue("ty"));
    SmartDashboard.putNumber("LimelightHelpers Y", LimelightHelpers.getLimelightNTDouble("limelight", "ty"));
    SmartDashboard.putNumber("Limelight X", getTableValue("tx"));
    SmartDashboard.putNumber("Limelight Area", getTableValue("ta"));
    SmartDashboard.putBoolean("Has Target", hasValidTarget());
    SmartDashboard.putNumber("Target Distance", getTargetDistance());
    SmartDashboard.putNumber("Distance (Limelight Helpers)", getTargetDistanceUsingLimelightHelper());
  }
  
  @Override
  public void periodic() {
    putLimelightValuesInDashboard(); // doesn't put values in smartdashboard -> periodic works for methods relating directly to subsystem
  }
}
