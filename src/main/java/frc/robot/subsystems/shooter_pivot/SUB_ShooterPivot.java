package frc.robot.subsystems.shooter_pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_ShooterPivot extends SubsystemBase {
    
    IO_ShooterPivotBase io;
    ShooterPivotInputsAutoLogged inputs = new ShooterPivotInputsAutoLogged();

    public SUB_ShooterPivot(IO_ShooterPivotBase io){
        this.io =io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterPivot", inputs);
    }

    public void changeDegreeAim(double degree) {
        io.changeDegreeAim(degree);
    }

    public void setPivotMotors(double degree) {
        io.setPivotMotors(degree);
    }
    
    public double getAbsoluteDegree(){
       return io.getAbsoluteDegree();        
    }

    public void fixedPos(){
        io.fixedPos();
    }

    public void limelightPos(){
        io.limelightPos();
    }

}
