package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeCollectorConstants;

public class AlgaeCollector extends SubsystemBase {
    
    private SparkMax algaeAdjusterMotor = new SparkMax(AlgaeCollectorConstants.ALGAE_ADJUSTER_MOTOR_ID, MotorType.kBrushless);
    private SparkMax algaeCollectorMotor = new SparkMax(AlgaeCollectorConstants.ALGAE_COLLECTOR_MOTOR_ID, MotorType.kBrushless);

    private RelativeEncoder algaeAdjusterEncoder = algaeAdjusterMotor.getEncoder();

    private SparkMaxConfig adjusterConfig = new SparkMaxConfig();
    private SparkMaxConfig collectorConfig = new SparkMaxConfig();

    private DigitalInput beamBreaker = new DigitalInput(AlgaeCollectorConstants.ALGEA_BEAM_BREAKER_PORT);
    private DigitalInput algaeCollectorLimitSwitch = new DigitalInput(AlgaeCollectorConstants.ALGAE_COLLECTOR_LIMIT_SWITCH_PORT);

    public AlgaeCollector() {
        adjusterConfig.inverted(true)
            .idleMode(IdleMode.kBrake);
        collectorConfig.inverted(false)
            .idleMode(IdleMode.kBrake);


        algaeAdjusterMotor.configure(adjusterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        algaeCollectorMotor.configure(collectorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        algaeAdjusterResetEncoderPos();
    }

    public void algaeAdjusterResetEncoderPos() {
        algaeAdjusterEncoder.setPosition(0);
    }

    public boolean hasAlgae() {
        return !algaeCollectorLimitSwitch.get();// returns opposite, !limitSwitch.get() ==> false
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

    /***** COMMANDS *****/
    public Command collectorUpwardCommand(double speed) {
        return run(() -> algaeAdjusterMotor.set(speed));
    }

    public Command collectorDownwardCommand(double speed) {
        return run(() -> algaeAdjusterMotor.set(-speed));
    }

    public Command grabAlgaeCommand(double speed) {
        return run(() -> algaeCollectorMotor.set(speed));
    }

    public Command grabAlgaeWithBeamBreakerCommand(double speed) {
        return run(() -> {
            while(!beamBreaker.get()) // might change to 'if'
                algaeCollectorMotor.set(speed);
            algaeCollectorMotor.stopMotor();
        });
    }

    public Command releaseAlgaeCommand(double speed) {
        return run(() -> {
            while(!beamBreaker.get())
                algaeCollectorMotor.set(-speed);
            algaeCollectorMotor.stopMotor();
        });

    }

    public Command stopAllCommand() {
        return run(() -> {
            algaeAdjusterMotor.stopMotor();
            algaeCollectorMotor.stopMotor();
        });
    }

    @Override
    public void periodic() {}
}
