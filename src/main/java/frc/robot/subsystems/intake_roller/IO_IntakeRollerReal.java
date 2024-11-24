package frc.robot.subsystems.intake_roller;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.utilities.Builder;

public class IO_IntakeRollerReal implements IO_IntakeRollerBase {

        CANSparkMax rollerMotor;

        DigitalInput limitSwitch = new DigitalInput(1);

        public IO_IntakeRollerReal(){
            rollerMotor = Builder.createNeo(Constants.IntakeConstants.ROLLER_MOTOR_PORT, false, 40);
            Builder.configureIdleMode(rollerMotor, true);
        }

        @Override
        public void updateInputs(IntakeRollerInputs inputs) {

            inputs.intakeRollerAppliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
            inputs.intakeRollerCurrentAmps = new double[] {rollerMotor.getOutputCurrent()};        
            inputs.limitSwitch = limitSwitch.get();
        }    

        @Override
        public void setRollerMotor(double speed){
        rollerMotor.setVoltage(MathUtil.clamp(12*speed, -12.0, 12.0));
      }

      @Override
      public boolean getLimitSwitch(){
        return limitSwitch.get();
      }
        

}

