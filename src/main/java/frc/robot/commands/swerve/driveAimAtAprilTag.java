package frc.robot.commands.swerve;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.SUB_Vision;

public class driveAimAtAprilTag extends Command {
  private final SUB_Swerve SwerveSub;
    private final SUB_Vision VisionSub;
  /** Creates a new driveAimmAtTarget. */
  public driveAimAtAprilTag(SUB_Swerve s_SwerveSubsystem, SUB_Vision s_VisionSubsystem) {
    this.SwerveSub = s_SwerveSubsystem;
    this.VisionSub = s_VisionSubsystem;
    addRequirements(s_SwerveSubsystem,VisionSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


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
