// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /* Can't access limelight as table object */
  // static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // static NetworkTableEntry tx = table.getEntry("tx");
  // static NetworkTableEntry ty = table.getEntry("ty");
  // static NetworkTableEntry ta = table.getEntry("ta");
  // static NetworkTableEntry tv = table.getEntry("tv");

  /** Creates a new Limelight. */
  public Limelight() {}

  public void setLimelightPipeline(double pipelineNumber) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(1);
  }

  public static double getLimelightTableValue(String tablentry) {
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
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0.0);
      default:
        return 0;
    }
  }

  public static boolean hasValidTarget() {
    return (getLimelightTableValue("tv") == 1) ? true : false;
  }

  public static double getTargetDistance() {

    double angleToGoalDegrees = Constants.Limelight.CAMERA_MOUNT_ANGLE_DEG + getLimelightTableValue("ty");
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
    double distanceFromLimelightToGoalInches = (Constants.Limelight.GOAL_HEIGHT_INCHES - Constants.Limelight.CAMERA_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

    double distanceToTarget = distanceFromLimelightToGoalInches - Constants.Limelight.CAMERA_TO_EDGE_OF_ROBOT_INCHES;

    return distanceToTarget;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Limelight Y", getLimelightTableValue("ty"));
    SmartDashboard.putNumber("Limelight X", getLimelightTableValue("tx"));
    SmartDashboard.putNumber("Limelight Area", getLimelightTableValue("ta"));
    SmartDashboard.putBoolean("Has Target", hasValidTarget());
    SmartDashboard.putNumber("Target Distance", getTargetDistance());
  }
}
