package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeCollectorConstants;

public class AlgaeCollector extends SubsystemBase {
    // algae position constants
    private final double kLowerPosition = 13.5;// 13.5
    private final double kStartPosition = 1; // offset start

    
    private SparkMax algaeAdjusterMotor = new SparkMax(AlgaeCollectorConstants.ALGAE_ADJUSTER_MOTOR_ID, MotorType.kBrushless);
    private SparkMax algaeCollectorMotor = new SparkMax(AlgaeCollectorConstants.ALGAE_COLLECTOR_MOTOR_ID, MotorType.kBrushless);

    private RelativeEncoder adjusterEncoder = algaeAdjusterMotor.getEncoder();

    private SparkMaxConfig adjusterConfig = new SparkMaxConfig();
    private SparkMaxConfig collectorConfig = new SparkMaxConfig();

    // closedloop for controlling motor
    private SparkClosedLoopController adjusterClosedLoopController = algaeAdjusterMotor.getClosedLoopController();

    private DigitalInput beamBreaker = new DigitalInput(AlgaeCollectorConstants.ALGAE_COLLECTOR_LIMIT_SWITCH_PORT);

    public AlgaeCollector() {
        adjusterConfig.inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);
        collectorConfig.inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30);

        adjusterConfig.closedLoop
            .p(0.1)
            .outputRange(-0.3, 0.4);

        algaeAdjusterMotor.configure(adjusterConfig, null, null);
        algaeCollectorMotor.configure(collectorConfig, null, null);

        algaeAdjusterResetEncoderPos();
    }

    public void algaeAdjusterResetEncoderPos() {
        adjusterEncoder.setPosition(0);
    }

    public boolean hasAlgae() {
        return !beamBreaker.get();// returns opposite, !limitSwitch.get() ==> false
    }

    public void collectAlgae(double speed) {
        algaeCollectorMotor.set(speed);
    }

    public void releaseAlgae(double speed) {
        algaeCollectorMotor.set(-speed);
    }

    public void algaeCollectorUp(double speed) {
        algaeAdjusterMotor.set(speed);
    }

    public void algaeCollectorDown(double speed) {
        algaeAdjusterMotor.set(-speed);
    }

    public void stopAlgaeCollector() {
        algaeCollectorMotor.stopMotor();
    }

    public void stopAlgeaAdjuster() {
        algaeAdjusterMotor.stopMotor();
    }

    public void stopMotors() {
        stopAlgaeCollector();
        stopAlgeaAdjuster();
    }


    /////////////////////////////// BOOLEAN ///////////////////////////////
    public boolean atStartPosition() {
        return getAdjusterPositionFloored() == kStartPosition;
    }

    public boolean atPickUpPosition() {
        return getAdjusterPositionFloored() == Math.floor(kLowerPosition); // modify
    }

    public boolean hasAlgaeAtStart() {
        return hasAlgae() && atStartPosition();
    }

    public double getAdjusterPosition() {
        return adjusterEncoder.getPosition();
    }

    public double getAdjusterPositionFloored() {
        return Math.floor(adjusterEncoder.getPosition());
    }

    public void lowerAlgaeAndCollect(double collectorSpeed) {
        if(!hasAlgae()) {
            algaeCollectorMotor.set(collectorSpeed);
            adjusterClosedLoopController.setReference(kLowerPosition, ControlType.kPosition);
        }
        if(hasAlgae()){
            adjusterClosedLoopController.setReference(kStartPosition, ControlType.kPosition);
            algaeCollectorMotor.stopMotor();
        }
    }

    public void lowerToCollect() {
        if(getAdjusterPositionFloored() != kLowerPosition) {
            adjusterClosedLoopController.setReference(kLowerPosition, ControlType.kPosition);
        }
    }

    public void adjustAlgaeToStart() {
        if(getAdjusterPosition() != kStartPosition)
            adjusterClosedLoopController.setReference(kStartPosition, ControlType.kPosition);
        else if(getAdjusterPosition() == kStartPosition)
            algaeAdjusterMotor.stopMotor();
    }

    @Override
    public void periodic() {}
}