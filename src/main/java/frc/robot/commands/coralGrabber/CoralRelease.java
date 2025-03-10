package frc.robot.commands.coralGrabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGrabber;

public class CoralRelease extends Command {
    private CoralGrabber coralGrabber;
    private double speed;

    public CoralRelease(CoralGrabber coralGrabber, double speed) {
        this.coralGrabber = coralGrabber;
        this.speed = speed;

        addRequirements(coralGrabber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        coralGrabber.releaseCoral(speed);
    }

    @Override
    public void end(boolean interrupted) {
        coralGrabber.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
