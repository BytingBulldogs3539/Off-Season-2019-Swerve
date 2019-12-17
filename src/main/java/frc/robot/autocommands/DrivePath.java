/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.SwerveModifier;
import jaci.pathfinder.*;

public class DrivePath extends Command
{
  SwerveModifier modifier;
  EncoderFollower flFollower,frFollower,blFollower, brFollower;

  public DrivePath(Trajectory source) {
    requires(Robot.driveTrain);
    modifier = new SwerveModifier(source).modify(0.5, 0.5, SwerveModifier.Mode.SWERVE_DEFAULT);
    
    flFollower = new EncoderFollower(modifier.getFrontLeftTrajectory());   // Front Left wheel
    flFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 13, 0);

    flFollower = new EncoderFollower(modifier.getFrontLeftTrajectory()); // Front Left wheel
    flFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 13, 0);

    flFollower = new EncoderFollower(modifier.getFrontLeftTrajectory()); // Front Left wheel
    flFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 13, 0);
    
    flFollower = new EncoderFollower(modifier.getFrontLeftTrajectory()); // Front Left wheel
    flFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 13, 0);

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
    double flOutput = flFollower.calculate((int)Robot.driveTrain.FLModule.getDriveMotor().getEncoder().getPosition());
    double flDesiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(flFollower.getHeading())); // Bound to -180..180
                                                                                                  // degrees

    Robot.driveTrain.FLModule.setAngle(flDesiredHeading);
    Robot.driveTrain.FLModule.setSpeed(flOutput);
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
}
