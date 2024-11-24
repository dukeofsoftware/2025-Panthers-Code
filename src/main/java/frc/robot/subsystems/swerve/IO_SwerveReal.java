
package frc.robot.subsystems.swerve;

import java.io.File;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.util.PIDConstants;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.utilities.LimelightHelpers;
import edu.wpi.first.wpilibj.Filesystem;

public class IO_SwerveReal implements IO_SwerveBase  {

  private static File jsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  SwerveDrive swerveDrive;
  private final Pigeon2 pigeonIMU;


  public IO_SwerveReal() 
  {

    try{
      swerveDrive = new SwerveParser(jsonDirectory).createSwerveDrive(Constants.MaxModuleSpeed);

    } catch (Exception e){
      throw new RuntimeException(e);
    }

   

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW; // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    swerveDrive.setAngularVelocityCompensation(true,
    true,
    0.1);
    pigeonIMU = (Pigeon2)swerveDrive.getGyro().getIMU();
    swerveDrive.stopOdometryThread();

  }
  
  	/**
	 * Retrieves the current swerve drive heading PID constants.
	 *
	 * @return The heading PID constants
	 */
	@Override
	public PIDConstants getHeadingPID() {

		return new PIDConstants(
				swerveDrive.swerveController.config.headingPIDF.p, // Rotation PID
				swerveDrive.swerveController.config.headingPIDF.i,
				swerveDrive.swerveController.config.headingPIDF.d);
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	@Override
	public void updateInputs(SwerveInputs inputs) {
		inputs.pose = swerveDrive.getPose();
		inputs.yaw = swerveDrive.getYaw();
		inputs.odometryHeading = swerveDrive.getOdometryHeading();

	}
  @Override
  public Rotation2d  getOdometryHeading(){
    return swerveDrive.getOdometryHeading();
  }

  @Override
  public SwerveController getSwerveController(){
    return swerveDrive.getSwerveController();
  }
  @Override
  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

   
  @Override
  public Rotation2d getYaw() {
      return swerveDrive.getYaw();
  }


	/** Sets the angle of all swerve modules to 0.0. */
	@Override
	public void setZero() {
		for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
			module.setAngle(0.0);
		}
	}
	/**
	 * Drive the robot using the given translation, rotation, and mode.
	 *
	 * @param translation The desired translation of the robot
	 * @param rotation The desired rotation of the robot
	 * @param isFieldRelative Whether the driving mode is field relative
	 * @param isOpenLoop Whether the driving mode is open loop
	 */
	@Override
	public void drive(
			Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop) {
		swerveDrive.drive(translation, rotation, isFieldRelative, isOpenLoop);
	}


 

/**
	 * Retrieves the configuration radius of the swerve drive.
	 *
	 * @return the configuration radius in meters
	 */
	@Override
	public double getConfigurationRadius() {
		return swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters();
	}
 
	/**
	 * Retrieves the maximum swerve angular velocity.
	 *
	 * @return The maximum angular velocity
	 */
	@Override
	public double getMaximumAngularVelocity() {
		return swerveDrive.getMaximumAngularVelocity();
	}
    
    
  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

   /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   * @return The robot's pose
   */
  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake){
    swerveDrive.setMotorIdleMode(brake);
  }
    /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity(){
    return swerveDrive.getFieldVelocity();
  }
  
  /**
   * Gets the current velocity (x, y and omega) of the robot
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity(){
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   * @return The yaw angle
   */
  public Rotation2d getHeading(){
    return getPose().getRotation();
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock(){
    swerveDrive.lockPose();
  }
  public double getMaxVelocity(){
    return swerveDrive.getMaximumVelocity();
  }

  public double getMaxAngularVelocity(){
    return swerveDrive.getMaximumAngularVelocity();
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

public void updateVisionOdometry(){
    boolean rejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight",swerveDrive.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if(Math.abs(pigeonIMU.getRate()) > 360){ // if our angular velocity is greater than 360 degrees per second, ignore vision updates
      rejectUpdate = true;
    }
    if(limelightMeasurement.tagCount == 0){
      rejectUpdate = true;
    }
    if(!rejectUpdate){
      swerveDrive.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
    }
  }

}

