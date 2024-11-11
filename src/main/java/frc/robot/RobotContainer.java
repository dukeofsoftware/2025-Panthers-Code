package frc.robot;

import frc.robot.subsystems.DriverControlsSubsystem;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
  

  private final SendableChooser<Command> autoChooser;
  private  final DriverControlsSubsystem driverControlsSubsystem;
 

  public RobotContainer() 
  {

    driverControlsSubsystem = DriverControlsSubsystem.getInstance();
  
    driverControlsSubsystem.registerTriggers();
    autoChooser = AutoBuilder.buildAutoChooser("Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

    

  public Command getAutonomousCommand() {
    
   return autoChooser.getSelected();
  }
}
