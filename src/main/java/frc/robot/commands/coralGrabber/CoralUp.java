// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.coralGrabber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CoralGrabber;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CoralUp extends Command {
//   private CoralGrabber coralGrabber;
//   private double speed;

//   /** Creates a new CoralUp. */
//   public CoralUp(CoralGrabber coralGrabber, double speed) {
//     this.coralGrabber = coralGrabber;
//     this.speed = speed;

//     addRequirements(coralGrabber);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     coralGrabber.up(speed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     coralGrabber.stopAdjuster();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return CoralGrabber.getCoralLimitSwitch_Up();
//   }
// }
