package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;



public class DriverControlsSubsystem extends SubsystemBase{
    
    static DriverControlsSubsystem instance;
    private SwerveSubsystem drivebase =  new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));

    private CommandXboxController driverController;
    private PS4Controller operatorController;
    private XboxController practiceController;
    private double speedRate;
  
    public DriverControlsSubsystem(){
        // kollar
        driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
        operatorController = new PS4Controller(Constants.OperatorConstants.kOperatorControllerPort);
        practiceController = new XboxController(Constants.OperatorConstants.kPracticeControllerPort);
     
        speedRate =1;
    }



      Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)*speedRate,
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)*speedRate,
        () ->  MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        ()->!driverController.rightBumper().getAsBoolean());


    public PS4Controller getOperatorController()
    {
        return operatorController;
    }
     public XboxController getPractiveController()
     {
        return practiceController;
     }

    


    public void registerTriggers()
    {

      driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.y().whileTrue(drivebase.aimAtSpeaker(2));
      driverController.b().whileTrue(
          Commands.deferredProxy(() -> drivebase.driveToPose(
                                     new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)))
                                ));
    driverController.back().whileTrue(drivebase.centerModulesCommand());

    driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    
    }
    public static DriverControlsSubsystem getInstance()
    {
        if(instance==null)
        {
            instance=new DriverControlsSubsystem();
        }
        return instance;
    }
        
    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {
    }


}