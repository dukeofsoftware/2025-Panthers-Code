package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  
  public static final RobotMode CURRENT_MODE = RobotMode.SIM;


  public static final double MaxModuleSpeed =5.3;
  public static final double driveBaseRadius = 0.51;

 public static class PIDConstants{
      public static final double kSwerveAutoPIDP = 15;
      public static final double kSwerveAutoPIDI = 0;
      public static final double kSwerveAutoPIDD = 0.1;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort=1;
    public static final int kPracticeControllerPort=2;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
  }

  public static class IntakeConstants {
    public static final int PIVOT_MOTOR_PORT = 9;
    public static final int ROLLER_MOTOR_PORT = 15;
    public static final double PIVOT_POWER = 0.3;
    public static final double AMP_SHOOT_POWER = 6;
    public static final double ROLLER_POWER = 8.5;
    public static final double INTAKE_P=0.009;
    public static final double INTAKE_I=0.00;
    public static final double INTAKE_D=0.0005;
    public static final int kCPR=8192;
  }

  public static class ShooterConstant {
    public static final int ROLLER_MOTOR1_PORT= 18;
    public static final int ROLLER_MOTOR2_PORT= 19;
    public static final int PIVOT_MOTOR1_PORT= 16;
    public static final int PIVOT_MOTOR2_PORT= 17;
    public static final double PIVOT_POWER=0.5;
    public static final double AMP_POWER=2.5; //11.5 speaker 2 amfi
    public static final double SPEAKER_POWER=11.5;
  }

  public static class ClimberConstant
  {
    public static final int CLIMBER1_PORT=20;
    public static final int CLIMBER2_PORT=21;
    public static final double CLIMBER_POWER=1;
  }
    public static class FieldConstants {
    public static final Translation2d kSpeakerPositionBLUE = new Translation2d(0.0, 5.547868); //TODO: Confirm these
    public static final Translation2d kSpeakerPositionRED = new Translation2d(16.5410642, 5.547868);
  }
  	// Current robot mode
	public enum RobotMode {
		REAL, // Running on a real robot
		SIM, // Running a physics simulator
		REPLAY // Replaying from a log file
	}
}
