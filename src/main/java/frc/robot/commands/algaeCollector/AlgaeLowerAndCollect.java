package frc.robot.commands.algaeCollector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;

public class AlgaeLowerAndCollect extends Command {

  private AlgaeCollector algaeCollector;
  private double collectorSpeed;

  public AlgaeLowerAndCollect(AlgaeCollector algaeCollector, double collectorSpeed) {
    this.algaeCollector = algaeCollector;

    addRequirements(algaeCollector);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    algaeCollector.lowerAlgaeAndCollect(collectorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    algaeCollector.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return algaeCollector.hasAlgaeAtStart();
  }
}
