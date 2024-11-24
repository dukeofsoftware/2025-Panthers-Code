package frc.robot.subsystems.intake_roller;

import org.littletonrobotics.junction.AutoLog;

public interface IO_IntakeRollerBase {

    @AutoLog
    public static class IntakeRollerInputs{
        public double intakeRollerAppliedVolts = 0.0;
        public double[] intakeRollerCurrentAmps = new double[] {};
        public boolean limitSwitch = false;
    }

    void updateInputs(IntakeRollerInputs inputs);
    
    public void setRollerMotor(double speed);
    public boolean getLimitSwitch();
} 
