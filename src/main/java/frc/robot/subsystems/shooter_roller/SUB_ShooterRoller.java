package frc.robot.subsystems.shooter_roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_ShooterRoller extends SubsystemBase{
 
        IO_ShooterRollerBase io;
        ShooterRollerInputsAutoLogged inputs = new ShooterRollerInputsAutoLogged();

       public SUB_ShooterRoller(IO_ShooterRollerBase io){
        this.io= io;

       }

       @Override
       public void periodic() {
       io.updateInputs(inputs);
       Logger.processInputs("ShooterRoller", inputs);
       }

       public void setRollerMotors(double speed){
        io.setRollerMotors(speed);
}
    

}
