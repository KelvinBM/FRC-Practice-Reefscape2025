// package frc.robot.commands.coralGrabber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CoralGrabber;

// public class CoralDown  extends Command{
//     private CoralGrabber coralGrabber;
//     private double speed;

//     public CoralDown(CoralGrabber coralGrabber, double speed) {
//         this.coralGrabber = coralGrabber;
//         this.speed = speed;

//         addRequirements(coralGrabber);
//     }

//     @Override
//     public void execute() {
//         coralGrabber.down(speed);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         coralGrabber.stopAdjuster();
//     }

//     @Override
//     public boolean isFinished(){
//         return CoralGrabber.getCoralLimitSwitch_Down();
//     }
// }
