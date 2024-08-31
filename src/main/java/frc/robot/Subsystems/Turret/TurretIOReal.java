package frc.robot.Subsystems.Turret;

// imports libraries
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

// the REAL motors (functions for turret motor)
public class TurretIOReal implements TurretIO{
    // instantiating + initializing the motors
    private final TalonFX turretMotor = new TalonFX(0, "canivore");
    private final CANcoder turretEncoder = new CANcoder(0, "canivore");
    private final PositionVoltage request = new PositionVoltage(0).withSlot(0); //?

    // instantiates the configuration (motor config - gear ratio, fwd, rev) of motor and encoder
    TalonFXConfiguration talonFXConfigs;
    CANcoderConfiguration canCoderConfigs;

    // constructor of class
    public TurretIOReal(){
        // initialization of the motor configuration
        talonFXConfigs = new TalonFXConfiguration();
        turretMotor.getConfigurator().apply(talonFXConfigs);

        // creating pid configs
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        CANcoderConfiguration cc_cfg = new CANcoderConfiguration(); // creates new CANcoderConfiguration
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // setting magnet absolute range
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // setting direction for encoder positions
        cc_cfg.MagnetSensor.MagnetOffset = 0; // sets the zero direction
        turretEncoder.getConfigurator().apply(cc_cfg); // applies configuration

        TalonFXConfiguration fx_cfg = new TalonFXConfiguration(); // creates new TalonFX configuration
        fx_cfg.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID(); // gets the ID of the encoder and sets it to the motor to indentify it
        fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // setting the source of input to fuse CAN coder (basically telling the robot it's a FusedCANcoder)
        fx_cfg.Feedback.SensorToMechanismRatio = 1.0; // the gear ratio between encoder and mechanism (turret) (1:1)
        fx_cfg.Feedback.RotorToSensorRatio = 60.0; // sets the ratio of the rotor and encoder (shaft of motor and encoder) 

        turretMotor.getConfigurator().apply(fx_cfg); // applies configuration

        // applying pid configs to motor
        turretMotor.getConfigurator().apply(slot0Configs);

        // initialization of the encoder configuration
        canCoderConfigs = new CANcoderConfiguration();
        turretEncoder.getConfigurator().apply(canCoderConfigs);
    }

    // overrides the update to the actual value of the motors
    @Override
    public void updateInputs(TurretIOInputs inputs){
        // set inputs of the turret motors
        inputs.turretRadPerSec = Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble()); // radians per sec velocity (rate of change)
        inputs.turretRad = Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble()); // radians position
        inputs.turretCurrent = turretMotor.getStatorCurrent().getValueAsDouble(); // current amps
        inputs.turretVoltage = turretMotor.getMotorVoltage().getValueAsDouble(); // voltage volts
    }
    @Override 
    public void setProfiled(double target, double additionalVoltage) {
        // Convert the target position from radians to rotations
        double targetRotations = Units.radiansToRotations(target);

        // Set the motor to move to the target position with additional voltage
        turretMotor.setControl(request.withPosition(targetRotations).withFeedForward(additionalVoltage)); // additional voltage is for feedforward
    }

    public void setProfiled(double target) {
        double targetRotations = Units.radiansToRotations(target);

        // Set the motor to move to the target position with additional voltage
        turretMotor.setControl(request.withPosition(targetRotations).withFeedForward(0)); // additional voltage is for feedforward
    }
    

    @Override
    public void setVoltage(double voltage) {
        turretMotor.setVoltage(voltage);
    }
}
