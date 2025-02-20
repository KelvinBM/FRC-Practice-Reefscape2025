// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralGrabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralGrabber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralStopAll extends InstantCommand {
  private CoralGrabber coralGrabber;

  /** Creates a new CoralStop. */
  public CoralStopAll(CoralGrabber coralGrabber) {
    this.coralGrabber = coralGrabber;

    addRequirements(coralGrabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralGrabber.stopAllMotors();
  }
}
