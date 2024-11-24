package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.utilities.Builder;

public class IO_ClimberReal implements IO_ClimberBase {
    
    private CANSparkMax climber1Motor;
    private CANSparkMax climber2Motor;


    public IO_ClimberReal(){

        climber1Motor = Builder.createNeo(Constants.ClimberConstant.CLIMBER1_PORT, false, 40);
        climber2Motor = Builder.createNeo(Constants.ClimberConstant.CLIMBER2_PORT, false, 40);
        Builder.configureIdleMode(climber1Motor, true);
        Builder.configureIdleMode(climber2Motor, true);
        
    }

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	@Override
	public void updateInputs(ClimberInputs inputs) {
        inputs.climber1AppliedVolts = climber1Motor.getAppliedOutput() * climber1Motor.getBusVoltage();
        inputs.climber1CurrentAmps = new double[] {climber1Motor.getOutputCurrent()};

        inputs.climber2AppliedVolts = climber2Motor.getAppliedOutput() * climber2Motor.getBusVoltage();
        inputs.climber2CurrentAmps = new double[] {climber2Motor.getOutputCurrent()};

	}

    @Override
    public void setClimber1Voltage(double speed) {
        climber1Motor.set(speed);
    }

   
    @Override
    public void setClimber2Voltage(double speed) {
        climber2Motor.set(speed);
    }
  

}
