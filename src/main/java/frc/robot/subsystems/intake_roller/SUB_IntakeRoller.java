package frc.robot.subsystems.intake_roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_IntakeRoller extends SubsystemBase {
    
    private final IO_IntakeRollerBase io;
    public final IntakeRollerInputsAutoLogged inputs = new IntakeRollerInputsAutoLogged();


    public SUB_IntakeRoller(IO_IntakeRollerBase io){
        this.io = io;
    }



    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeRoller", inputs);
    }

    public boolean getLimitSwitch(){
        return io.getLimitSwitch();
    }

    public void setRollerMotor(double speed){
        io.setRollerMotor(speed);
    }
}
