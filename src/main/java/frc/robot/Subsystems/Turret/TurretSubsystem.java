package frc.robot.Subsystems.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase{

    private final TurretIO io; 
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final double turretAngleRadians = 45; // TODO: measure angle that turret can turn

    public TurretSubsystem(TurretIO io) {
        this.io = io;
    }

    @Override
    // runs every 20 msec (update inputs, etc.)
    public void periodic() {
        io.updateInputs(inputs); // gets inputs inputs from motor + updates them
        Logger.processInputs("Flywheels", inputs); // logs all the angles that the turret turns to
    }

    // 0 center, left negative, right positive
    // current*voltage = power
    public void setAngle(double radiansAngle) {
        if (Math.abs(radiansAngle) <= radiansAngle) io.setProfiled(radiansAngle); // sets the voltage for the turret
    }

    public Command setAngleCommand(double angle) {
        return this.runOnce(() -> setAngle(angle));
    }

    public Command leftCommand() {
        return this.runOnce(() -> setAngle(-turretAngleRadians));
    }
    
    public Command rightCommand() {
        return this.runOnce(() -> setAngle(turretAngleRadians));
    }
    
    public Command zeroCommand() {
        return this.runOnce(() -> setAngle(0));
    }
    
    public Command randomCommand() {
        return this.runOnce(() -> setAngle(Math.random()*2*turretAngleRadians + -turretAngleRadians));
    }
}
