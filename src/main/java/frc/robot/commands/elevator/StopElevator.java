package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class StopElevator extends Command {
    private Elevator elevator;

    public StopElevator(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stopAllMotors();
    }


    @Override
    public void end(boolean interrupted) {
        elevator.stopAllMotors();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}