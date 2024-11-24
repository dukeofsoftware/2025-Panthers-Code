package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class IO_VisionReal implements IO_VisionBase {

        AprilTagFieldLayout fieldLayout;

    public IO_VisionReal(){
try {
    fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

} catch (Exception e) {
    e.printStackTrace();
    }
    }

    

    @Override
    public void updateInputs(VisionInputs inputs) {
        
    }

}
