package frc.robot.commands.algaeCollector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;

public class StopAlgaeCollector extends Command {
    private AlgaeCollector algaeCollector;

    public StopAlgaeCollector(AlgaeCollector algaeCollector) {
        this.algaeCollector = algaeCollector;
        addRequirements(algaeCollector);
    }

    @Override
    public void initialize() {
        algaeCollector.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
