package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;


public interface IO_VisionBase {

    @AutoLog
    public static class VisionInputs{
        
    }
    void updateInputs(VisionInputs inputs);
    
    
}