// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StopAll;
import frc.robot.commands.algaeCollector.AlgaeAdjustToStart;
import frc.robot.commands.algaeCollector.AlgaeLowerAndCollect;
import frc.robot.commands.algaeCollector.AlgaeLowerToCollect;
import frc.robot.commands.elevator.ElevatorLevel1;
import frc.robot.commands.elevator.ElevatorLevel2;
import frc.robot.commands.elevator.ElevatorLevel3;
import frc.robot.commands.elevator.LowerElevator;
import frc.robot.commands.elevator.RaiseElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeCollector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralGrabber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick buttonBoard = new Joystick(OperatorConstants.BUTTON_BOARD_PORT);
    // private final CommandJoystick
    // private final JoystickButton elevatorLevel1, elevatorLevel2, elevatorLevel3, 
    //                             elevatorStartPosition, algaeLowerAndCollect, algaeStartPosition,
    //                             algeaAllInOne, algeaRelease, coralGrab, coralRelease, 
    //                             stopAll;
    private final JoystickButton stopAll, raiseElevator, lowerElevator, elevatorL1, elevatorL2, elevatorL3;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Climber climber = new Climber();
    public final AlgaeCollector algaeCollector = new AlgaeCollector();
    public final CoralGrabber coralGrabber = new CoralGrabber();
    public final Limelight limelight = new Limelight();


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);


        // button bindings
        elevatorL1 = new JoystickButton(buttonBoard, OperatorConstants.PORT_1);
        elevatorL2 = new JoystickButton(buttonBoard, OperatorConstants.PORT_2);
        elevatorL3 = new JoystickButton(buttonBoard, OperatorConstants.PORT_3);
        raiseElevator = new JoystickButton(buttonBoard, OperatorConstants.PORT_4);
        lowerElevator = new JoystickButton(buttonBoard, OperatorConstants.PORT_5);
        stopAll = new JoystickButton(buttonBoard, OperatorConstants.PORT_6);
        

        configureBindings();
    }

    private void configureBindings() {
        // assign buttons their ports
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                driveFieldCentric.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().onTrue(new AlgaeLowerAndCollect(algaeCollector, 0.3).andThen(new AlgaeAdjustToStart(algaeCollector)));
        joystick.b().onTrue(new StopAll(algaeCollector, climber, coralGrabber, elevator));
        joystick.y().onTrue(new AlgaeLowerToCollect(algaeCollector));
        joystick.x().onTrue(new AlgaeAdjustToStart(algaeCollector));

        stopAll.onTrue(new StopAll(algaeCollector, climber, coralGrabber, elevator));
        raiseElevator.whileTrue(new RaiseElevator(elevator, 0.2));
        lowerElevator.whileTrue(new LowerElevator(elevator, 0.2));
        
        elevatorL1.onTrue(new ElevatorLevel1(elevator));
        elevatorL2.onTrue(new ElevatorLevel2(elevator));
        elevatorL3.onTrue(new ElevatorLevel3(elevator));

        // l1.onTrue(new)
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));


        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
