package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class SUB_Swerve extends SubsystemBase {

	private final IO_SwerveBase io;
	private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();
	// Odometry lock, prevents updates while reading data
	private static final Lock odometryLock = new ReentrantLock();



	public SUB_Swerve(IO_SwerveBase io) {
		this.io = io;
		try {

		} catch (Exception e) {
			// TODO: handle exception
		}

		AutoBuilder.configureHolonomic(
				io::getPose, // Gets current robot pose
				io::resetOdometry, // Resets robot odometry if path has starter pose
				io::getRobotVelocity, // Gets chassis speed in robot relative
				io::setChassisSpeeds, // Drives the robot in robot relative chassis speeds
				new HolonomicPathFollowerConfig(
						new PIDConstants(5.0, 0.0, 0.0, 0.0), // Translation
						io.getHeadingPID(),
						/*
						 * IMPORTANT NOTE: These auto PIDs only have a relatively small effect. For a larger and better controlled one use the YAGSL PIDF config.
						 * When setting these PID values make sure rotation is faster than translation and there is a feedforward on the YAGSL PIDF config.
						 */
						Constants.MaxModuleSpeed,
						io.getConfigurationRadius(), // Drive base radius in meters
						new ReplanningConfig() // Replanning config see docs
						),
				() -> {
					// Auto path flipper for alliance color, always make paths on blue side
					var alliance = DriverStation.getAlliance();
					return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
				},
				this);


	}

	public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX,BooleanSupplier fieldOrianted)
	{
	  return run(() -> {
		// Make the robot move
		DriverStation.Alliance color;
		  color = DriverStation.getAlliance().get();
		int isBlueAlliance= ((DriverStation.Alliance.Red==color)&& fieldOrianted.getAsBoolean()==true )? -1:1;
		io.drive(new Translation2d(-translationX.getAsDouble() * io.getMaxVelocity()*isBlueAlliance,
											-translationY.getAsDouble() * io.getMaxVelocity()*isBlueAlliance),
						  angularRotationX.getAsDouble() * io.getMaximumAngularVelocity(),
						  fieldOrianted.getAsBoolean(),
						  false);
	  });
	}



	public void headingDrive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY){
		double xInput = translationX.getAsDouble();
		double yInput = translationY.getAsDouble();
	
		if(isAllianceBlue()){
		  driveFieldOriented(io.getSwerveController().getTargetSpeeds(xInput, yInput, headingX.getAsDouble(), headingY.getAsDouble(), io.getOdometryHeading().getRadians(), io.getMaxVelocity()));
		}else{
		  driveFieldOriented(io.getSwerveController().getTargetSpeeds(-xInput, -yInput, headingX.getAsDouble(), headingY.getAsDouble(), io.getOdometryHeading().getRadians(), io.getMaxVelocity()));
		}
	  }

    
	/**
	 * Drives the robot to the specified pose.
	 *
	 * @param pose The target pose to drive to
	 * @return The command to run to drive to the pose.
	 */
	public Command driveToPose(Pose2d pose) {
		Command pathfindingCommand =
				AutoBuilder.pathfindToPose(
						pose,
						new PathConstraints(
								4.0, // autos in PathPlanner is 5.5
								3.0, // autos in PathPlanner is 3.3
								Units.degreesToRadians(540), // 540 350
								Units.degreesToRadians(720) // 720 // 540
								));
		return pathfindingCommand;
	}

  /**
   * Drive the robot given a chassis field oriented velocity.
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity){
    io.driveFieldOriented(velocity);
  }

	/**
	 * Drives the robot on the path specified.
	 *
	 * @param name The name of the path to follow
	 * @param setOdomToStart Whether or not to reset the odometry to the start of the path
	 * @return The command to run to drive the path
	 */
	public Command drivePath(String name, boolean setOdomToStart) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(name);
		if (setOdomToStart) {
			io.resetOdometry(new Pose2d(path.getPoint(0).position, io.getHeading()));
		}
		return AutoBuilder.followPath(path);
	}
	
	public Command findPathAndDrive(String name){
		PathPlannerPath path = PathPlannerPath.fromPathFile(name);
		PathConstraints constraints = new PathConstraints(io.getMaxVelocity(), Constants.MaxModuleSpeed, Units.degreesToRadians(io.getMaxAngularVelocity()), Units.degreesToRadians(720));

		return AutoBuilder.pathfindThenFollowPath(path, constraints);
	}

	public Translation2d getTargetSpeaker(){ 
		if(isAllianceBlue()){
		  return Constants.FieldConstants.kSpeakerPositionBLUE;
		}else{
		  return Constants.FieldConstants.kSpeakerPositionRED;
		}
	  }

	  public boolean isAllianceBlue(){ 
	  var alliance = DriverStation.getAlliance();
	  if(alliance.isPresent()){
		if(alliance.get() == DriverStation.Alliance.Blue){
		  return true;
		}else{
		  return false;
		}
	  }else{
		return true;
	  }
	}

	
  
	public void resetEmergencyOdometry() {
		io.resetOdometry(new Pose2d(0.0, 0.0, io.getYaw()));
	}

	public Pose2d getPose() {
		return io.getPose();
	}

	public Rotation2d getYaw() {
		return io.getYaw();
	}

    public void zeroGyro(){
        io.zeroGyro();
    }
	

	public void periodic() {
		
		io.updateVisionOdometry();
		odometryLock.lock();
		io.updateInputs(inputs);
		odometryLock.unlock();

		Logger.processInputs("Swerve", inputs);
	
	}

}
