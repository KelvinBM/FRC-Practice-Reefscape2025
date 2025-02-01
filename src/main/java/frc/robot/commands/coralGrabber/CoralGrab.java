package frc.robot.commands.coralGrabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGrabber;

public class CoralGrab extends Command {
    private CoralGrabber coralGrabber;
    private double speed;

    public CoralGrab(CoralGrabber coralGrabber, double speed) {
        this.coralGrabber = coralGrabber;
        this.speed = speed;

        addRequirements(coralGrabber);
    }

    @Override
    public void execute() {
        coralGrabber.grabCoral(speed);
    }

    @Override
    public void end(boolean interrupted) {
        coralGrabber.stopLeftMotor();
        coralGrabber.stopRightMotor();
    }

    @Override
    public boolean isFinished() {
        return CoralGrabber.hasCoral();
    }
}
