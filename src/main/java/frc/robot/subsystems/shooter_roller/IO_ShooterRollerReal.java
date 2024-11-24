package frc.robot.subsystems.shooter_roller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.utilities.Builder;

public class IO_ShooterRollerReal implements IO_ShooterRollerBase {
    
    CANSparkMax flywheel1;
    CANSparkMax flywheel2;

    RelativeEncoder flywheel1Encoder;
    RelativeEncoder flywheel2Encoder;


    public IO_ShooterRollerReal(){

            flywheel1 = Builder.createNeo(Constants.ShooterConstant.ROLLER_MOTOR1_PORT, false, 40);
            flywheel2 = Builder.createNeo(Constants.ShooterConstant.ROLLER_MOTOR2_PORT, true, 40);

            
            flywheel1Encoder= flywheel1.getEncoder();
            flywheel2Encoder=flywheel2.getEncoder();
        }


    @Override
    public void updateInputs(ShooterRollerInputs inputs) {
        inputs.shooterRoller1AppliedVolts = flywheel1.getAppliedOutput() * flywheel1.getBusVoltage();
        inputs.shooterRoller1CurrentAmps = new double[] {flywheel1.getOutputCurrent()};      

        inputs.shooterRoller2AppliedVolts = flywheel2.getAppliedOutput() * flywheel2.getBusVoltage();
        inputs.shooterRoller2CurrentAmps = new double[] {flywheel2.getOutputCurrent()};   

        inputs.flywheel1Velocity=flywheel2Encoder.getVelocity();
        inputs.flywheel2Velocity=flywheel2Encoder.getVelocity();
    }
    public void setRollerMotors(double speed){
                flywheel1.setVoltage(MathUtil.clamp(12*speed, -12.0, 12.0));
                flywheel2.setVoltage(MathUtil.clamp(12*speed, -12.0, 12.0));
    }

}
