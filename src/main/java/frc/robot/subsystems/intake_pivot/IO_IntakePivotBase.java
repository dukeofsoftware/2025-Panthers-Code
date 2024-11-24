package frc.robot.subsystems.intake_pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IO_IntakePivotBase {
    

    @AutoLog
    public static class IntakePivotInputs{
        public double intakePivotPositionRad = 0.0;
        public double intakePivotVelocityRadPerSec = 0.0;
        public double intakePivotAppliedVolts = 0.0;
        public double[] intakePivotCurrentAmps = new double[] {};
        public double intakePivotPositionRotations;
    }
    void updateInputs(IntakePivotInputs inputs);
    public double getPosition();
    public void setPivotMotor(double speed);
    public void setPivotPID(Rotation2d angle);
}
