/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.swerve_library.math.CentricMode;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveCommand extends Command
{
  public DriveCommand()
  {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize()
  {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute()
  {
    double x = 0;
    double y = 0;
    double x1 = 0;

    if(Robot.oi.driver.buttonBR.get() && Robot.driveTrain.swerveDrive.getCentricMode()==CentricMode.FIELD)
    {
      Robot.driveTrain.swerveDrive.setCentricMode(CentricMode.ROBOT);
    }
    else if(Robot.driveTrain.swerveDrive.getCentricMode() != CentricMode.FIELD && !Robot.oi.driver.buttonBR.get())
    {
      Robot.driveTrain.swerveDrive.setCentricMode(CentricMode.FIELD);
    }

    if (Robot.oi.driver.buttonA.get())
    {
      Robot.driveTrain.setAllAngles(0);
    }
    else
    {

      if (Robot.oi.driver.getLeftStickY() >= .05 || Robot.oi.driver.getLeftStickY() <= -.05)
        y = Robot.oi.driver.getLeftStickY() * 1.0;
      else
        y = 0;

      if (Robot.oi.driver.getLeftStickX() >= .05 || Robot.oi.driver.getLeftStickX() <= -.05)
        x = Robot.oi.driver.getLeftStickX() * 1.0;
      else
        x = 0;

      if (Robot.oi.driver.getRightStickX() >= .05 || Robot.oi.driver.getRightStickX() <= -.05)
        x1 = Robot.oi.driver.getRightStickX() * 1.0;
      else
        x1 = 0;

      Robot.driveTrain.drive(y, x, x1);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished()
  {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end()
  {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted()
  {
  }
}
