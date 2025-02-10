package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeCollector extends SubsystemBase {
    
    private SparkMax algaeAdjusterMotor = new SparkMax(Constants.ALGEA_ADJUSTER_MOTOR_ID, MotorType.kBrushless);
    private SparkMax algaeCollectorMotor = new SparkMax(Constants.ALGEA_COLLECTOR_MOTOR_ID, MotorType.kBrushless);

    private SparkMaxConfig adjusterConfig = new SparkMaxConfig();
    private SparkMaxConfig collectorConfig = new SparkMaxConfig();

    private DigitalInput beamBreaker = new DigitalInput(Constants.ALGEA_BEAM_BREAKER_PORT);

    public AlgaeCollector() {
        adjusterConfig.inverted(true)
            .idleMode(IdleMode.kBrake);
        collectorConfig.inverted(false)
            .idleMode(IdleMode.kBrake);

        algaeAdjusterMotor.configure(adjusterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        algaeCollectorMotor.configure(collectorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
