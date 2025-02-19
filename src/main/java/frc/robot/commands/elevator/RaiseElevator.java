package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RaiseElevator extends Command {

    private Elevator elevator;
    private double speed;

    public RaiseElevator(Elevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.raiseElevator(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopAllMotors();
    }
}
