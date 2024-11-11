
package frc.robot.subsystems;

import java.io.File;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.utilities.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {

  SwerveDrive RobotSwerve;

  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final Pigeon2 pigeonIMU;


  public SwerveSubsystem(File directory) 
  {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW; // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.

    try{
      RobotSwerve = new SwerveParser(directory).createSwerveDrive(Constants.MaxModuleSpeed);
    } catch (Exception e){
      throw new RuntimeException(e);
    }
    RobotSwerve.setHeadingCorrection(false);
    RobotSwerve.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    RobotSwerve.setAngularVelocityCompensation(true,
    true,
    0.1);
    setupPathPlanner();
    pigeonIMU = (Pigeon2)RobotSwerve.getGyro().getIMU();

  }
  


  public void setupPathPlanner()
  {
  AutoBuilder.configureHolonomic(
              this::getPose,
              this::resetOdometry,
              this::getRobotVelocity,
              this::setChassisSpeeds,
              new HolonomicPathFollowerConfig(
                  new PIDConstants(Constants.PIDConstants.kSwerveAutoPIDP, Constants.PIDConstants.kSwerveAutoPIDI, Constants.PIDConstants.kSwerveAutoPIDD),
                  new PIDConstants(
                      RobotSwerve.swerveController.config.headingPIDF.p,
                      RobotSwerve.swerveController.config.headingPIDF.i,
                      RobotSwerve.swerveController.config.headingPIDF.d),
                  Constants.MaxModuleSpeed,
                  RobotSwerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                  new ReplanningConfig()
              ),
              () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
              },
              this
          );  
  }

    public Command getAutonomousCommand(String pathName){
    return new PathPlannerAuto(pathName);
  }

    /** Use PathPlanner Path finding to go to a point on the field.
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command*/
  public Command driveToPose(Pose2d pose){
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(RobotSwerve.getMaximumVelocity(), 4.0,RobotSwerve.getMaximumAngularVelocity(), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(pose, constraints,0.0, 0.0);
  }
    /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return Commands.deferredProxy(
        () -> Commands.run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)), this)
                      .until(() -> RobotSwerve.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                                   distanceInMeters)
                                 );
  }


//MAVİ TARAFTA ÇALIŞIYOR
public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX,BooleanSupplier fieldOrianted)
{
  return run(() -> {
    // Make the robot move
    DriverStation.Alliance color;
    color = DriverStation.getAlliance().get();
    int isBlueAlliance= ((DriverStation.Alliance.Red==color)&& fieldOrianted.getAsBoolean()==true )? -1:1;
    //SmartDashboard.putNumber("Joystick", angularRotationX.getAsDouble());
    RobotSwerve.drive(new Translation2d(-translationX.getAsDouble() * RobotSwerve.getMaximumVelocity()*isBlueAlliance,
                                        -translationY.getAsDouble() * RobotSwerve.getMaximumVelocity()*isBlueAlliance),
                      angularRotationX.getAsDouble() * RobotSwerve.getMaximumAngularVelocity(),
                      fieldOrianted.getAsBoolean(),
                      false);
  });
}
    /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(RobotSwerve.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

    /**
   * The primary method for controlling the drivebase. 
   * 
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in m/s. In robot-relative mode, positive x is torwards front and positive y is torwards left.  In field-relative mode, positive x is away from the alliance wall (field North) and positive y is torwards the left wall when looking through the driver station glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot, relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative){
    RobotSwerve.drive(translation, rotation, fieldRelative,false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void headingDrive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY){
    double xInput = translationX.getAsDouble();
    double yInput = translationY.getAsDouble();

    if(isAllianceBlue()){
      driveFieldOriented(RobotSwerve.swerveController.getTargetSpeeds(xInput, yInput, headingX.getAsDouble(), headingY.getAsDouble(), RobotSwerve.getOdometryHeading().getRadians(), RobotSwerve.getMaximumVelocity()));
    }else{
      driveFieldOriented(RobotSwerve.swerveController.getTargetSpeeds(-xInput, -yInput, headingX.getAsDouble(), headingY.getAsDouble(), RobotSwerve.getOdometryHeading().getRadians(), RobotSwerve.getMaximumVelocity()));
    }
  }
    /**
   * Drive the robot given a chassis field oriented velocity.
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity){
    RobotSwerve.driveFieldOriented(velocity);
  }
    /**
   * Drive according to the chassis robot oriented velocity.
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity){
    RobotSwerve.drive(velocity);
  }
  
   /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      // Make the robot move
      driveFieldOriented(RobotSwerve.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                      translationY.getAsDouble(),
                                                                      rotation.getAsDouble() * Math.PI,
                                                                      RobotSwerve.getOdometryHeading().getRadians(),
                                                                      RobotSwerve.getMaximumVelocity()));
    });
  }
   /**
   * Aim the robot at the speaker.
   *
   * @param tolerance Tolerance in degrees.
   * @return Command to turn the robot to the speaker.
   */
  public Command aimAtSpeaker(double tolerance)
  {
    SwerveController controller = RobotSwerve.getSwerveController();
    return run(
        () -> {
          drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,
                                                      0,
                                                      controller.headingCalculate(getHeading().getRadians(),
                                                                                  getSpeakerYaw().getRadians()),
                                                      getHeading())
               );
        }).until(() -> Math.abs(getSpeakerYaw().minus(getHeading()).getDegrees()) < tolerance);
  }
  
  public Rotation2d getSpeakerYaw()
  {
    int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    // Taken from PhotonUtils.getYawToPose()
    Pose3d        speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
    Translation2d relativeTrl         = speakerAprilTagPose.toPose2d().relativeTo(getPose()).getTranslation();
    return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(RobotSwerve.getOdometryHeading());
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose){
    RobotSwerve.resetOdometry(initialHolonomicPose);
  }
   /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   * @return The robot's pose
   */
  public Pose2d getPose(){
    return RobotSwerve.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
    RobotSwerve.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro(){
    RobotSwerve.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake){
    RobotSwerve.setMotorIdleMode(brake);
  }
    /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity(){
    return RobotSwerve.getFieldVelocity();
  }
  
  /**
   * Gets the current velocity (x, y and omega) of the robot
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity(){
    return RobotSwerve.getRobotVelocity();
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
    RobotSwerve.lockPose();
  }
  public double getMaxVelocity(){
    return RobotSwerve.getMaximumVelocity();
  }

  public double getMaxAngularVelocity(){
    return RobotSwerve.getMaximumAngularVelocity();
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
    LimelightHelpers.SetRobotOrientation("limelight",RobotSwerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if(Math.abs(pigeonIMU.getRate()) > 360){ // if our angular velocity is greater than 360 degrees per second, ignore vision updates
      rejectUpdate = true;
    }
    if(limelightMeasurement.tagCount == 0){
      rejectUpdate = true;
    }
    if(!rejectUpdate){
      RobotSwerve.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
    }
  }
  

 @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  

}

