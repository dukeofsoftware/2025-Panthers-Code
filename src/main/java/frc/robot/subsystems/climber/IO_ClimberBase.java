package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ClimberBase {

    @AutoLog
    public static class ClimberInputs{
        public double climber1AppliedVolts = 0.0;
        public double[] climber1CurrentAmps = new double[] {};
        
        public double climber2AppliedVolts = 0.0;
        public double[] climber2CurrentAmps = new double[] {};
    }
    
    void updateInputs(ClimberInputs inputs);

    public void setClimber1Voltage(double speed);
	public void setClimber2Voltage(double speed);    
} 