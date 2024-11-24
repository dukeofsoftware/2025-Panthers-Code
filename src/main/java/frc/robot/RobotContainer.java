package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.climber.IO_ClimberReal;
import frc.robot.subsystems.climber.SUB_Climber;
import frc.robot.subsystems.intake_pivot.IO_IntakePivotReal;
import frc.robot.subsystems.intake_pivot.SUB_IntakePivot;
import frc.robot.subsystems.intake_roller.IO_IntakeRollerReal;
import frc.robot.subsystems.intake_roller.SUB_IntakeRoller;
import frc.robot.subsystems.shooter_pivot.IO_ShooterPivotReal;
import frc.robot.subsystems.shooter_pivot.SUB_ShooterPivot;
import frc.robot.subsystems.shooter_roller.IO_ShooterRollerReal;
import frc.robot.subsystems.shooter_roller.SUB_ShooterRoller;
import frc.robot.subsystems.swerve.IO_SwerveReal;
import frc.robot.subsystems.swerve.SUB_Swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  

  private final SendableChooser<Command> autoChooser;
  private double speedRate;
  private  CommandXboxController  driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private PS4Controller operatorController = new PS4Controller(Constants.OperatorConstants.kOperatorControllerPort);


  private final SUB_Swerve swerve =  new SUB_Swerve(new IO_SwerveReal());

  private final SUB_Climber climber =	new SUB_Climber(new IO_ClimberReal());

  private final SUB_IntakePivot intakePivot = new SUB_IntakePivot(new IO_IntakePivotReal());
  private final SUB_IntakeRoller intakeRoller = new SUB_IntakeRoller(new IO_IntakeRollerReal());


  private final SUB_ShooterPivot shooterPivot = new SUB_ShooterPivot(new IO_ShooterPivotReal());
  private final SUB_ShooterRoller shooterRoller = new SUB_ShooterRoller(new IO_ShooterRollerReal());
 

 
  public RobotContainer() 
  {
    speedRate=1;
  registerTriggers();
    autoChooser = AutoBuilder.buildAutoChooser("Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
		logMetadata();
  }





     public void registerTriggers()
    {

    new Trigger(()-> driverController.getRawAxis(4) > 0).onTrue(new InstantCommand(()->speedRate=0.2))
    .onFalse(new InstantCommand(()->speedRate=1));

   // driverController.y().whileTrue(new driveAimAtSpeakerPose(swerve, () -> MathUtil.applyDeadband(-driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), () -> MathUtil.applyDeadband(-driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)));
    //driverController.a().onTrue(new InstantCommand(swerve::zeroGyro));
    //driverController.x().whileTrue(swerve.drivePath("Example Path",false));
   
    new Trigger(()->operatorController.getR2Button()).onTrue(new InstantCommand(()->climber.climber1Motor(Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->climber.climber1Motor(0)));
    new Trigger(()->operatorController.getR1Button()).onTrue(new InstantCommand(()->climber.climber1Motor(-Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->climber.climber1Motor(0)));
    new Trigger(()->operatorController.getL2Button()).onTrue(new InstantCommand(()->climber.climber2Motor(Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->climber.climber2Motor(0)));
    new Trigger(()->operatorController.getL1Button()).onTrue(new InstantCommand(()->climber.climber2Motor(-Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->climber.climber2Motor(0)));



      //new Trigger(()-> driverController.getRawAxis(5) > 0).onTrue(new CMD_ClimbClose(climb, () -> 1.0));
     // new Trigger(()-> driverController.getRawAxis(4)>0).onTrue(new CMD_ClimbOpen(climb, () -> 1.0));


    swerve.setDefaultCommand(swerve.driveCommand(() -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND)*speedRate,
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)*speedRate,
        () ->  MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        ()->!driverController.rightBumper().getAsBoolean()));
    
    }

  	public void logMetadata() {
		Logger.recordMetadata("Event Name", DriverStation.getEventName());
		Logger.recordMetadata("Driver Station Location", DriverStation.getLocation() + "");
		Logger.recordMetadata("Match Number", DriverStation.getMatchNumber() + "");
		Logger.recordMetadata("Match Type", DriverStation.getMatchType() + "");
		Logger.recordMetadata("Replay Number", DriverStation.getReplayNumber() + "");
		Logger.recordMetadata("Robot Mode", "" + Constants.CURRENT_MODE);
	}
  
  public Command getAutonomousCommand() {
    
   return autoChooser.getSelected();
  }
}
