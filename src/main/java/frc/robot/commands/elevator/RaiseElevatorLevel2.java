package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RaiseElevatorLevel2 extends Command {
    private Elevator elevator;
    private double speed;

    public RaiseElevatorLevel2(Elevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;

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
