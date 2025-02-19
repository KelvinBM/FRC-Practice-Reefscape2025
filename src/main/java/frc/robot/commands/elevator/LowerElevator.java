package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class LowerElevator extends Command {
    private Elevator elevator;
    private double speed;

    public LowerElevator(Elevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.lowerElevator(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopAllMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
