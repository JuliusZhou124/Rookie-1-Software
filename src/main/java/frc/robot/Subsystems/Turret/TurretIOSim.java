package frc.robot.Subsystems.Turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.TurretConstants;

// Simulation of turret movement
public class TurretIOSim implements TurretIO{
    // copied from last year's code
    SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getFalcon500(1), TurretConstants.kMotorGearing, 0.05, 0.5, Math.toRadians(-50), Math.toRadians(50), false, Math.toRadians(0));
    double voltage = 0;

    @Override
    public void updateInputs(TurretIOInputs inputs){
        // Simulatting all the variables for turret
        sim.update(0.02);
        inputs.turretRad = sim.getAngleRads();
        inputs.turretRadPerSec = sim.getVelocityRadPerSec();
        inputs.turretVoltage = voltage;
        inputs.turretCurrent = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage){
        // setting voltage for simulation
        this.voltage = voltage;
        sim.setInputVoltage(voltage);
    }

    @Override
    public void setProfiled(double angle) {
        
    }
}
