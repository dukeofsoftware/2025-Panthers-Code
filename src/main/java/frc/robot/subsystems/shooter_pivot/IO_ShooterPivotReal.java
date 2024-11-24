package frc.robot.subsystems.shooter_pivot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ShooterConstant;
import frc.robot.utilities.Builder;
import frc.robot.utilities.LimelightHelpers;

public class IO_ShooterPivotReal implements IO_ShooterPivotBase {
    
    private PIDController shooterPID;

        CANSparkMax pivotMotor1;
        CANSparkMax pivotMotor2;

        boolean isLimelightActive = true;

        static DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(9);
        
        public double degreeAim;

        public IO_ShooterPivotReal(){
          pivotMotor1= Builder.createNeo(ShooterConstant.PIVOT_MOTOR1_PORT, true, 40);
          pivotMotor2= Builder.createNeo(ShooterConstant.PIVOT_MOTOR2_PORT, false, 40);
        absoluteEncoder.reset();

        shooterPID = new PIDController(6,0,0);

        PIDinitialize(0.965);
        }

        private void PIDinitialize(double degree)
        {
            degreeAim = degree;
            shooterPID.reset();
            shooterPID.setSetpoint(degree);
            shooterPID.setTolerance(0.0001);
        }

        @Override
        public void updateInputs(ShooterPivotInputs inputs) {
            inputs.isLimelightActive= isLimelightActive;
            inputs.degreeAim=degreeAim;

            inputs.shooterPivot1AppliedVolts =pivotMotor1.getAppliedOutput() * pivotMotor1.getBusVoltage();
             inputs.shooterPivot1CurrentAmps = new double[] {pivotMotor1.getOutputCurrent()};

            inputs.shooterPivot2AppliedVolts =pivotMotor2.getAppliedOutput() * pivotMotor2.getBusVoltage();
             inputs.shooterPivot2CurrentAmps =new double[] {pivotMotor2.getOutputCurrent()};
        }

     @Override
      public void changeDegreeAim(double degree) {
        PIDinitialize(degree);
    }

    @Override
    public void setPivotMotors(double degree) {
        
        pivotMotor1.set(degree);
        pivotMotor2.set(degree);
    }

    @Override
    public double getAbsoluteDegree()
    {
        double degree=(absoluteEncoder.getAbsolutePosition()<0.5f)? absoluteEncoder.getAbsolutePosition()+1:absoluteEncoder.getAbsolutePosition() ;
        return degree;
    }

    @Override
    public void fixedPos()
    {
        isLimelightActive=false;
        PIDinitialize(0.965);
    }

    @Override
    public void limelightPos()
    {
        isLimelightActive=true;
        PIDinitialize(DISTANCE_TO_ANGLE_MAP.get(LimelightHelpers.getTY("limelight")));
    }
        
   public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();

    static {
        DISTANCE_TO_ANGLE_MAP.put(14.2, 0.965);
        DISTANCE_TO_ANGLE_MAP.put(16.89, 0.965);
        DISTANCE_TO_ANGLE_MAP.put(11.75,0.97);
        DISTANCE_TO_ANGLE_MAP.put(9.45,0.976);
        DISTANCE_TO_ANGLE_MAP.put(7.45,0.98); 
        DISTANCE_TO_ANGLE_MAP.put(5.59,0.984);
        DISTANCE_TO_ANGLE_MAP.put(3.91,0.988);
        DISTANCE_TO_ANGLE_MAP.put(2.3,0.99);
        DISTANCE_TO_ANGLE_MAP.put(0.76,0.995); 
        DISTANCE_TO_ANGLE_MAP.put(-0.68,0.997); 
        DISTANCE_TO_ANGLE_MAP.put(-1.91,0.999); 
        DISTANCE_TO_ANGLE_MAP.put(-3.20,1.0); 
        DISTANCE_TO_ANGLE_MAP.put(-4.3,1.005);
        DISTANCE_TO_ANGLE_MAP.put(-5.42,1.005);
        DISTANCE_TO_ANGLE_MAP.put(-6.4,1.008);
        DISTANCE_TO_ANGLE_MAP.put(-7.3,1.01);
        DISTANCE_TO_ANGLE_MAP.put(-8.2,1.013);  
        //DISTANCE_TO_ANGLE_MAP.put(3.0, ArmConstants.kOffset - 0.058);
        //DISTANCE_TO_ANGLE_MAP.put(4.1, ArmConstants.kOffset - 0.038);
        //DISTANCE_TO_ANGLE_MAP.put(4.9, ArmConstants.kOffset - 0.035);
        //DISTANCE_TO_ANGLE_MAP.put(5.5, ArmConstants.kOffset - 0.028);
      }
}


 
  
