// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralGrabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGrabber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralDownWithEncoder extends Command {
  private CoralGrabber coralGrabber;
  private double speed, targetEncoderPosition;
  private boolean finished;

  /** Creates a new CoralDown. */
  public CoralDownWithEncoder(CoralGrabber coralGrabber, double speed, double targetEncoderPosition) {
    this.coralGrabber = coralGrabber;
    this.speed = speed;
    this.targetEncoderPosition = targetEncoderPosition;

    addRequirements(coralGrabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    coralGrabber.resetAdjustEncoderPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = coralGrabber.downWithEncoder(speed, targetEncoderPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralGrabber.stopAdjuster();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
