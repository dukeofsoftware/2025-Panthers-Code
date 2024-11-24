package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SUB_Swerve;

public class driveAimAtSpeakerPose extends Command {
  private final SUB_Swerve SwerveSub;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private Translation2d targetSpeaker;
  private double absoluteHeading;
  /** Creates a new driveAimmAtTarget. */
  public driveAimAtSpeakerPose(SUB_Swerve s_SwerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {
    this.SwerveSub = s_SwerveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    targetSpeaker = new Translation2d();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetSpeaker = SwerveSub.getTargetSpeaker();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //no clue if this works, hell if I know
    //this somehow works actually, no clue how tbh
    double distanceX = SwerveSub.getPose().getX() - targetSpeaker.getX();
    double distanceY = SwerveSub.getPose().getY() - targetSpeaker.getY();
    double relativeHeading = Math.toDegrees(Math.atan2(distanceY, distanceX)); 

    absoluteHeading = Math.toRadians(-relativeHeading+270); //the magic sauce
    SwerveSub.headingDrive(translationX, translationY, ()-> Math.cos(absoluteHeading), ()-> Math.sin(absoluteHeading));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
