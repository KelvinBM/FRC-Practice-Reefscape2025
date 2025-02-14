package frc.robot.commands.algaeCollector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;

public class ReleaseAlgae extends Command {
    
    private AlgaeCollector algaeCollector;
    private double speed;

    public ReleaseAlgae(AlgaeCollector algaeCollector, double speed) {
        this.algaeCollector = algaeCollector;
        this.speed = speed;

        addRequirements(algaeCollector);
    }


    @Override
    public void execute() {
        algaeCollector.releaseAlgae(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        algaeCollector.stopAlgaeCollector();
    }

}
