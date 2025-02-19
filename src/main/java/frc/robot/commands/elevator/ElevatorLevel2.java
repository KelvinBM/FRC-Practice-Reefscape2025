package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorLevel2 extends Command {
    private Elevator elevator;

    public ElevatorLevel2(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.goToLevel2();
    }

    @Override
    public boolean isFinished() {
        return elevator.hasReachedLevel2();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopAllMotors();
    }

}
