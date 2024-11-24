
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import swervelib.SwerveController;

import org.littletonrobotics.junction.AutoLog;

import com.pathplanner.lib.util.PIDConstants;

public interface IO_SwerveBase {

	@AutoLog
	public static class SwerveInputs {
		public Pose2d pose = new Pose2d();
		public Rotation2d yaw = new Rotation2d();
		public Rotation2d odometryHeading = new Rotation2d();
	}

	void updateInputs(SwerveInputs inputs);

  Rotation2d  getOdometryHeading();

  SwerveController  getSwerveController();

	PIDConstants getHeadingPID();

  double getConfigurationRadius();

  void drive(Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop);  
  void driveFieldOriented(ChassisSpeeds velocity);
        
  void resetOdometry(Pose2d initialHolonomicPose);
  
  Pose2d getPose();
  
  void setChassisSpeeds(ChassisSpeeds chassisSpeeds);
  
  void setZero();

  void zeroGyro();

  void setMotorBrake(boolean brake);
  
  ChassisSpeeds getFieldVelocity();
  
  ChassisSpeeds getRobotVelocity();
  
  Rotation2d getHeading();
  
  Rotation2d getYaw();

  void lock();

  double getMaxVelocity();

  double getMaxAngularVelocity();

  boolean isAllianceBlue();
  

void updateVisionOdometry();

public double getMaximumAngularVelocity() ;
}
