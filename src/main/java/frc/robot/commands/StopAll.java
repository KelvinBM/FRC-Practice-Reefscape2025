// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralGrabber;
import frc.robot.subsystems.Elevator;

public class StopAll extends Command {
  private AlgaeCollector algaeCollector;
  private Climber climber;
  private CoralGrabber coralGrabber;
  private Elevator elevator;

  public StopAll(AlgaeCollector algaeCollector, Climber climber, CoralGrabber coralGrabber, Elevator elevator) {
    this.algaeCollector = algaeCollector;
    this.climber = climber;
    this.coralGrabber = coralGrabber;
    this.elevator = elevator;

    addRequirements(algaeCollector, climber, coralGrabber, elevator);
  }

  @Override
  public void initialize() {
    algaeCollector.stopMotors();
    climber.stopAllMotors();
    coralGrabber.stopAllMotors();
    elevator.stopAllMotors();
    System.out.println("STOPPED ALL");
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
