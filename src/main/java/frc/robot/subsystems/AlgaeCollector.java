package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeCollectorConstants;

public class AlgaeCollector extends SubsystemBase {
    // algae position constants
    private final double kLowerPosition = 16.5;// 16.5
    private final double kStartPosition = 2; // offset start
    private final double kHumanStationPosition = 0;

    
    private SparkMax algaeAdjusterMotor = new SparkMax(AlgaeCollectorConstants.ALGAE_ADJUSTER_MOTOR_ID, MotorType.kBrushless);
    private SparkMax algaeCollectorMotor = new SparkMax(AlgaeCollectorConstants.ALGAE_COLLECTOR_MOTOR_ID, MotorType.kBrushless);

    private RelativeEncoder adjusterEncoder = algaeAdjusterMotor.getEncoder();

    private SparkMaxConfig adjusterConfig = new SparkMaxConfig();
    private SparkMaxConfig collectorConfig = new SparkMaxConfig();

    // closedloop for controlling motor
    private SparkClosedLoopController adjusterClosedLoopController = algaeAdjusterMotor.getClosedLoopController();

    private DigitalInput beamBreaker = new DigitalInput(AlgaeCollectorConstants.ALGEA_BEAM_BREAKER_PORT);
    private DigitalInput algaeCollectorLimitSwitch = new DigitalInput(AlgaeCollectorConstants.ALGAE_COLLECTOR_LIMIT_SWITCH_PORT);

    public AlgaeCollector() {
        adjusterConfig.idleMode(IdleMode.kBrake);
        collectorConfig.inverted(true)
            .idleMode(IdleMode.kBrake);

        algaeAdjusterMotor.configure(adjusterConfig, null, null);
        algaeCollectorMotor.configure(collectorConfig, null, null);

        algaeAdjusterResetEncoderPos();
    }

    public void algaeAdjusterResetEncoderPos() {
        adjusterEncoder.setPosition(0);
    }

    public boolean hasAlgae() {
        return !algaeCollectorLimitSwitch.get();// returns opposite, !limitSwitch.get() ==> false
    }

    public void collectAlgae(double speed) {
        if(!hasAlgae())
            algaeCollectorMotor.set(speed);
        if(hasAlgae())
            algaeCollectorMotor.stopMotor();
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
        return getAdjusterPosition() == kStartPosition;
    }

    public boolean atPickUpPosition() {
        return getAdjusterPosition() == kLowerPosition;
    }

    public double getAdjusterPosition() {
        return Math.floor(adjusterEncoder.getPosition());
    }

    public void lowerAlgaeAndCollect(double collectorSpeed) {
        if(!hasAlgae())
            algaeCollectorMotor.set(collectorSpeed);
        if(hasAlgae())
            algaeCollectorMotor.stopMotor();
        if(getAdjusterPosition() != kLowerPosition)
            adjusterClosedLoopController.setReference(kLowerPosition, ControlType.kPosition);
        else if(getAdjusterPosition() == kLowerPosition)
            algaeAdjusterMotor.stopMotor();
    }

    public void lowerToCollect() {
        if(getAdjusterPosition() != kLowerPosition) {
            adjusterClosedLoopController.setReference(kLowerPosition, ControlType.kPosition);
        }
        if(getAdjusterPosition() == kLowerPosition) {
            algaeAdjusterMotor.stopMotor();
        }
    }

    public void adjustAlgaeToStart() {
        if(getAdjusterPosition() != kStartPosition)
            adjusterClosedLoopController.setReference(kStartPosition, ControlType.kPosition);
        else if(getAdjusterPosition() == kStartPosition)
            algaeAdjusterMotor.stopMotor();
    }

    public void adjustAlgaeToHuman() {
        if(getAdjusterPosition() != kHumanStationPosition) {
            adjusterClosedLoopController.setReference(kHumanStationPosition, ControlType.kPosition);
        } else if(getAdjusterPosition() == kHumanStationPosition)
            algaeAdjusterMotor.stopMotor();
    }

    @Override
    public void periodic() {}
}
