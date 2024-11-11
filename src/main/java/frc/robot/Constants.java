package frc.robot;


public final class Constants {

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
}
