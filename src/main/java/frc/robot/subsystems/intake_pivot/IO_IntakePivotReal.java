package frc.robot.subsystems.intake_pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utilities.Builder;

public class IO_IntakePivotReal implements IO_IntakePivotBase{
        private static double kS = 0.00;
        private static double kG = 0.2;
        private static double kV = 0.0; 

        CANSparkMax pivotMotor;
        RelativeEncoder pivotEncoder;
        SparkPIDController pivotPIDController;
        ArmFeedforward pivotFeedForward;

        static double setPoint;

    public IO_IntakePivotReal(){
        pivotMotor = Builder.createNeo(IntakeConstants.PIVOT_MOTOR_PORT, true, 40);
        Builder.configureIdleMode(pivotMotor, true);
        pivotPIDController= pivotMotor.getPIDController();

        pivotEncoder = pivotMotor.getAlternateEncoder(Type.kQuadrature,8192);
                pivotPIDController.setFeedbackDevice(pivotEncoder);

        pivotEncoder.setPositionConversionFactor(180);
        pivotEncoder.setPosition(-5);
        
        pivotFeedForward = new ArmFeedforward(kS, kG, kV);

        Builder.configurePIDController(pivotPIDController, false, new PIDConstants(
            Constants.IntakeConstants.INTAKE_P,
            Constants.IntakeConstants.INTAKE_I,
        Constants.IntakeConstants.INTAKE_D),
        0.0,0.0);

    }
    
    @Override
    public void setPivotPID(Rotation2d angle){
        pivotPIDController.setReference(angle.getDegrees(),
        CANSparkMax.ControlType.kPosition,
        0,
        pivotFeedForward.calculate(angle.getRadians(),0.00),
        ArbFFUnits.kVoltage);
        
       setPoint=angle.getDegrees();
    }
    
    @Override
    public void setPivotMotor(double speed) {
        pivotMotor.setVoltage(MathUtil.clamp(12*speed, -12.0, 12.0));    }

    @Override
    public double getPosition(){
        return pivotEncoder.getPosition();
    }
    
    @Override
    public void updateInputs(IntakePivotInputs inputs) {
        inputs.intakePivotPositionRad = Units.rotationsToRadians(pivotEncoder.getPosition());
        inputs.intakePivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.getVelocity());
        inputs.intakePivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.intakePivotCurrentAmps = new double[] {pivotMotor.getOutputCurrent()};
        inputs.intakePivotPositionRotations = pivotEncoder.getPosition();
        
    }


}
