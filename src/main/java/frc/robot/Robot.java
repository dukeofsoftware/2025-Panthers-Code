// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;


import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();



    Logger.recordMetadata("Maven Name", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("Build Date", BuildConstants.BUILD_DATE);

	if (Constants.CURRENT_MODE == Constants.RobotMode.REAL) {
	
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

		} else {
			setUseTiming(false); // Run as fast as possible
			/* FOR LOG REPLAY
			String logPath =
					LogFileUtil
							.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
			Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			Logger.addDataReceiver(
					new WPILOGWriter(
							LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
			*/
			Logger.addDataReceiver(new NT4Publisher());
         for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }
		}

		Logger.start();

   
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {

  }

  @Override
  public void disabledPeriodic() 
  {
    
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() 
  {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {

  }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


    m_robotContainer.logMetadata();

  }

  @Override
  public void teleopPeriodic() 
  {

  }

  @Override
  public void testInit() 
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() 
  {

  }

  @Override
  public void simulationInit() 
  {

  }

  @Override
  public void simulationPeriodic() 
  {

  }
}
