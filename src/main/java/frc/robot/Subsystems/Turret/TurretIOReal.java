package frc.robot.Subsystems.Turret;

import org.littletonrobotics.junction.Logger;

// imports libraries
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

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
        configureTalonFX();
        configureCANcoder();
    }

    private void configureTalonFX() {
        // initialization of the motor configuration
        talonFXConfigs = new TalonFXConfiguration();

        // creating pid configs for turret motor
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.0;  // Feedforward gain for static friction
        slot0Configs.kA = 0.0;  // Feedforward gain for acceleration
        slot0Configs.kV = 0.0; // Feedforward gain for velocity  // A velocity target of 1 rps results in 5 V output
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        // enable stator (output) current limit for turret motor
        var limitConfigs = talonFXConfigs.CurrentLimits;
        limitConfigs.StatorCurrentLimit = 20;
        limitConfigs.SupplyCurrentLimit = 20;
        limitConfigs.StatorCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimitEnable = false;

        talonFXConfigs.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID(); // gets the ID of the encoder and sets it to the motor to indentify it
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // setting the source of input to fuse CAN coder (basically telling the robot it's a FusedCANcoder)
        talonFXConfigs.Feedback.SensorToMechanismRatio = 1.0; // the gear ratio between encoder and mechanism (turret) (1:1)
        talonFXConfigs.Feedback.RotorToSensorRatio = 60.0; // sets the ratio of the rotor and encoder (shaft of motor and encoder)

        // **Motion Magic Configuration (commented out, use for planned motions)** :p (better PID)
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 2000;
        motionMagicConfigs.MotionMagicAcceleration = 5000;
        motionMagicConfigs.MotionMagicJerk = 0;//4000;

        // Motor Output Configs
        MotorOutputConfigs turretTalonOutputConfigs = new MotorOutputConfigs();
        turretTalonOutputConfigs.PeakForwardDutyCycle = 1;
        turretTalonOutputConfigs.PeakReverseDutyCycle = 1;
        turretTalonOutputConfigs.DutyCycleNeutralDeadband = 0.0;
        turretTalonOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        // Closed-Loop Ramps
        ClosedLoopRampsConfigs voltageRampConfig = new ClosedLoopRampsConfigs();
        voltageRampConfig.VoltageClosedLoopRampPeriod = 0;

        //important????
        var limitSwitchConfigs = new HardwareLimitSwitchConfigs();
        limitSwitchConfigs.ReverseLimitEnable = false;
        limitSwitchConfigs.ForwardLimitEnable = false;

        talonFXConfigs.withMotorOutput(turretTalonOutputConfigs);
        talonFXConfigs.withClosedLoopRamps(voltageRampConfig);
        talonFXConfigs.withCurrentLimits(talonFXConfigs.CurrentLimits);
        talonFXConfigs.withHardwareLimitSwitch(limitSwitchConfigs);


        turretMotor.getConfigurator().apply(talonFXConfigs); // applies configuration
        turretMotor.setInverted(true);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        turretMotor.setControl(new StaticBrake());

    }

    private void configureCANcoder() {
        // initialization of the encoder configuration
        canCoderConfigs = new CANcoderConfiguration();
        turretEncoder.getConfigurator().apply(canCoderConfigs);

        CANcoderConfiguration cc_cfg = new CANcoderConfiguration(); // creates new CANcoderConfiguration
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // setting magnet absolute range
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // setting direction for encoder positions
        cc_cfg.MagnetSensor.MagnetOffset = 0; // sets the zero direction
        turretEncoder.getConfigurator().apply(cc_cfg); // applies configuration
    }

    // overrides the update to the actual value of the motors
    @Override
    public void updateInputs(TurretIOInputs inputs){
        // set inputs of the turret motors
        inputs.turretRad = Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble()); // radians position
        inputs.turretRadPerSec = Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble()); // radians per sec velocity (rate of change)
        inputs.turretVoltage = turretMotor.getMotorVoltage().getValueAsDouble(); // voltage volts
        inputs.turretCurrent = turretMotor.getStatorCurrent().getValueAsDouble(); // current amps
    }
    @Override
    public void setProfiled(double target, double additionalVoltage) {
        Logger.recordOutput("RealOutputs/Turret/TargetRadiansMotionMagic", target);
        // Convert the target position from radians to rotations
        double targetRotations = Units.radiansToRotations(target);

        Logger.recordOutput("RealOutputs/Turret/TargetRotationMotionMagic", targetRotations);
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
