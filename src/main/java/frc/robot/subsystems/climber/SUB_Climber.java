package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Climber extends SubsystemBase {

    private final IO_ClimberBase io;
    public final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    public SUB_Climber(IO_ClimberBase io){
        this.io = io;
   
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }
 
    public void climber1Motor(double speed){
        io.setClimber1Voltage(speed); 
    }
    
    public void climber2Motor(double speed){
         io.setClimber2Voltage(speed);   
    }

}
