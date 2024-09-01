package frc.robot.Subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public double turretRad;
        public double turretRadPerSec;

        public double turretVoltage = 0.0;
        public double turretCurrent = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {}
    public default void setProfiled(double target, double additionalVoltage) {}
    public default void setProfiled(double target) {}
    public default void setVoltage(double voltage) {}
}
