package frc.robot.Subsystems.Turret;

// imports libraries
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

// the REAL motors (functions for turret motor)
public class TurretIOReal implements TurretIO{
    // instantiating + initializing the motors
    private final TalonFX turretMotor = new TalonFX(0, "canivore");
    private final CANcoder turretEncoder = new CANcoder(0, "canivore");

    // instantiates the configuration (motor config - gear ratio, fwd, rev) of motor and encoder
    TalonFXConfiguration talonFXConfigs;
    CANcoderConfiguration canCoderConfigs;

    // constructor of class
    public TurretIOReal(){
        // initialization of the motor configuration
        talonFXConfigs = new TalonFXConfiguration();
        turretMotor.getConfigurator().apply(talonFXConfigs);

        // initialization of the encoder configuration
        canCoderConfigs = new CANcoderConfiguration();
        turretEncoder.getConfigurator().apply(canCoderConfigs);
    }

    // overrides the update to the actual value of the motors
    @Override
    public void updateInputs(TurretIOInputs inputs){
        // set inputs of the turret motors
        inputs.turretRadPerSec = Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble()); // raidans per sec velocity (rate of change)
        inputs.turretRad = Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble()); // radians position
        inputs.turretCurrent = turretMotor.getStatorCurrent().getValueAsDouble(); // current amps
        inputs.turretVoltage = turretMotor.getMotorVoltage().getValueAsDouble(); // voltage volts
    }
}
