package frc.robot.Subsystems.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Turret.TurretIO.TurretIOInputs;

public class TurretSubsystem extends SubsystemBase{

    private final TurretIO io; 
    private final TurretIOInputs inputs = new TurretIOInputs();

    public TurretSubsystem(TurretIO io) {
        this.io = io;
    }

    @Override
    // runs every 20 msec (update inputs, etc.)
    public void periodic() {
        io.updateInputs(inputs); // gets inputs inputs from motor + updates them
    }

    // 0 center, left negative, right positive
    // current*voltage = power
    public void setAngle(double radiansAngle) {
        io.setProfiled(radiansAngle); // sets the voltage for the turret
    }

    public Command setAngleCommand(double angle) {
        return this.runOnce(() -> setAngle(angle));
    }
}
