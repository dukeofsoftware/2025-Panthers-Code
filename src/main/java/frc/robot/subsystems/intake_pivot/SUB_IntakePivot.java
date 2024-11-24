package frc.robot.subsystems.intake_pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_IntakePivot extends SubsystemBase {
    
    private final IO_IntakePivotBase io;
    public final IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();

    public SUB_IntakePivot(IO_IntakePivotBase io){
    this.io =io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);
    }

    public void setPivotPID(Rotation2d angle){
        io.setPivotPID(angle);
    }

    public void setPivotMotor(double speed) {
        io.setPivotMotor(speed);
    }
        
    public void getPosition(){
        io.getPosition();
    }



    

}
