package frc.robot.subsystems.shooter_roller;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ShooterRollerBase {
    
    @AutoLog
    public static class ShooterRollerInputs{
        public double shooterRoller1AppliedVolts = 0.0;
        public double[] shooterRoller1CurrentAmps = new double[] {};
        public double shooterRoller2AppliedVolts = 0.0;
        public double[] shooterRoller2CurrentAmps = new double[] {};
        public double flywheel1Velocity=0;
        public double flywheel2Velocity=0;
    }
    
    void updateInputs(ShooterRollerInputs inputs);
    public void setRollerMotors(double speed);

}
