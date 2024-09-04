package frc.robot.Subsystems.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        Logger.processInputs("Turret", inputs); // logs all the angles that the turret turns to
    }

    @Override
    public void simulationPeriodic() {
        io.updateInputs(inputs); // gets inputs inputs from motor + updates them
        Logger.processInputs("Turret", inputs); // logs all the angles that the turret turns to
        if (this.getCurrentCommand() != null )Logger.recordOutput("Current Turret Command", this.getCurrentCommand().getName());
        else Logger.recordOutput("Current Turret Command", "OOps");
    }

    // 0 center, left negative, right positive
    // current*voltage = power
    public void setAngle(double radiansAngle) {
        if (Math.abs(radiansAngle) <= radiansAngle) io.setProfiled(radiansAngle); // sets position for
    }

    public Command setAngleCommand(double angle) {
        return this.run(() -> setAngle(angle));
    }

    public Command leftCommand() {
        return this.run(() -> setAngle(-turretAngleRadians));
    }

    public Command rightCommand() {
        return this.run(() -> setAngle(turretAngleRadians));
    }

    public Command zeroCommand() {
        return this.run(() -> setAngle(0));
    }

    public Command randomCommand() {
        return this.run(() -> setAngle(Math.random()*2*turretAngleRadians + -turretAngleRadians));
    }

    public Command fCommand(double voltage) {
        return this.run(() -> io.setVoltage(voltage));
    }
}
