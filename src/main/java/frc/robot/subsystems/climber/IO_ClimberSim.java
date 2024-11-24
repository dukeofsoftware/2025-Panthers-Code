package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class IO_ClimberSim implements IO_ClimberBase {
    private DCMotorSim climber1 = new DCMotorSim(DCMotor.getNeoVortex(1), 25.0,0.00041);
    private DCMotorSim climber2 = new DCMotorSim(DCMotor.getNeoVortex(1), 25.0,0.00041);
    private double climber1AppliedVolts =0.0;
    private double climber2AppliedVolts =0.0;


    @Override
    public void updateInputs(ClimberInputs inputs) {
        climber1.update(0.02);
        climber2.update(0.02);
    
        inputs.climber1AppliedVolts = climber1AppliedVolts;
        inputs.climber1CurrentAmps = new double[] {climber1.getCurrentDrawAmps()};
    
        inputs.climber2AppliedVolts = climber2AppliedVolts;
        inputs.climber2CurrentAmps = new double[] {climber2.getCurrentDrawAmps()};

    }
    @Override
    public void setClimber1Voltage(double speed) {
        climber1AppliedVolts = MathUtil.clamp(12*speed, -12.0, 12.0);
        climber1.setInputVoltage(climber1AppliedVolts);        
    }
    @Override
    public void setClimber2Voltage(double speed) {
        climber2AppliedVolts = MathUtil.clamp(12*speed, -12.0, 12.0);
        climber2.setInputVoltage(climber2AppliedVolts);
    }
  

 

}
