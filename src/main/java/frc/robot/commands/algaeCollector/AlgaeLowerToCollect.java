package frc.robot.commands.algaeCollector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;

public class AlgaeLowerToCollect extends Command {
    private AlgaeCollector algaeCollector;

    public AlgaeLowerToCollect(AlgaeCollector algaeCollector) {
        this.algaeCollector = algaeCollector;

        addRequirements(algaeCollector);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        algaeCollector.lowerToCollect();
    }

    @Override
    public boolean isFinished() {
        return algaeCollector.atPickUpPosition();
    }

    @Override
    public void end(boolean interrupted) {
        algaeCollector.stopMotors();
    }
}
