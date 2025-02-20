// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algaeCollector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;

public class AlgaeAdjustToStart extends Command {

  private AlgaeCollector algaeCollector;

  public AlgaeAdjustToStart(AlgaeCollector algaeCollector) {
    this.algaeCollector = algaeCollector;

    addRequirements(algaeCollector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    algaeCollector.adjustAlgaeToStart();
  }

  @Override
  public void end(boolean interrupted) {
    algaeCollector.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return algaeCollector.atStartPosition();
  }
}
