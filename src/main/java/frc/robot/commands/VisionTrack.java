/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;


public class VisionTrack extends PIDCommand {

  public VisionTrack() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(1, 0, 0);
    requires(Robot.driveTrain);
    this.setSetpoint(.32);
    //pidController = new PIDController(.00001, Ki, Kd, source, output);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
  
  @Override
  protected double returnPIDInput() {
    return Robot.driveTrain.vision.get();
  }

  @Override
  protected void usePIDOutput(double output) {
    double x, y;
    if (Robot.oi.driver.getLeftStickY() >= .05 || Robot.oi.driver.getLeftStickY() <= -.05)
      y = Robot.oi.driver.getLeftStickY() * .25;
    else
      y = 0;

    if (Robot.oi.driver.getLeftStickX() >= .05 || Robot.oi.driver.getLeftStickX() <= -.05)
      x = Robot.oi.driver.getLeftStickX() * .25;
    else
      x = 0;

      
    if(output>.25)
    {
      output = .25;
    }
    Robot.driveTrain.drive(y, x, -output);
  }
}
