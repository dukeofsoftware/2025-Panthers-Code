package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.LimelightHelpers;

public class SUB_Vision extends SubsystemBase {

        private final IO_VisionBase io;
    	AprilTagFieldLayout fieldLayout;


    public SUB_Vision(IO_VisionBase io){
        this.io = io;
    }



    @Override
    public void periodic() {
          double error=LimelightHelpers.getTY("limelight");
        SmartDashboard.putNumber("LimelightTx", error);

    }

    


        
}
