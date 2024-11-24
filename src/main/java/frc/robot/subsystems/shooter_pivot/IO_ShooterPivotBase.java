package frc.robot.subsystems.shooter_pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ShooterPivotBase {
    
        @AutoLog
        public static class ShooterPivotInputs{
                public boolean isLimelightActive= false;
                public double degreeAim=0;

                public double shooterPivot1AppliedVolts =0.0;
                public double[] shooterPivot1CurrentAmps =  new double[] {};

                public double shooterPivot2AppliedVolts =0.0;
                public double[] shooterPivot2CurrentAmps = new double[] {};

        }

        void updateInputs(ShooterPivotInputs inputs);
        public void changeDegreeAim(double degree);
        public void setPivotMotors(double degree);
        public double getAbsoluteDegree();
        public void fixedPos();
        public void limelightPos();
}
