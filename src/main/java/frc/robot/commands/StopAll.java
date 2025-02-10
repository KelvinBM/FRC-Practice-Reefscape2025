// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralGrabber;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopAll extends Command {
  private Elevator elevator;
  private CoralGrabber coralGrabber;
  private Climber climber;

  /** Creates a new StopAll. */
  public StopAll(
    Elevator elevator, 
    CoralGrabber coralGrabber, 
    Climber climber) {

      this.elevator = elevator;
      this.coralGrabber = coralGrabber;
      this.climber = climber;

      addRequirements(elevator, coralGrabber, climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator. stopAllMotors();
    coralGrabber.stopAllMotors();
    climber.stopAllMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
