package frc.robot.commands.coralGrabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGrabber;

public class ReleaseCoral extends Command {
    private CoralGrabber coralGrabber;
    private double speed, timerStart;
    private boolean finished;

    public ReleaseCoral(CoralGrabber coralGrabber, double speed) {
        this.coralGrabber = coralGrabber;
        this.speed = speed;

        addRequirements(coralGrabber);
    }

    @Override
    public void initialize() {
        timerStart = Timer.getFPGATimestamp();
        finished = false;
    }

    @Override
    public void execute() {
        if(Timer.getFPGATimestamp() - timerStart <= 3)
            coralGrabber.releaseCoral(speed);
        else
            finished = true;
    }

    @Override
    public void end(boolean interrupted) {
        coralGrabber.stopLeftMotor();
        coralGrabber.stopRightMotor();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
