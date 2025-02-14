package frc.robot.commands.algaeCollector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;

public class CollectAlgae extends Command {
    private AlgaeCollector algaeCollector;
    private double speed;


    public CollectAlgae(AlgaeCollector algeaCollector, double speed) {
        this.algaeCollector = algeaCollector;
        this.speed = speed;

        addRequirements(algaeCollector);
    }

    @Override
    public void execute() {
        algaeCollector.collectAlgae(speed);
    }

    @Override 
    public boolean isFinished() {
        return algaeCollector.hasAlgae();
    }

    @Override
    public void end(boolean interrupted) {
        algaeCollector.stopAlgaeCollector();
    }
}
