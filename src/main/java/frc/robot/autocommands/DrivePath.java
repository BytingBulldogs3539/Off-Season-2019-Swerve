/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.SwerveModifier;
import jaci.pathfinder.*;

public class DrivePath extends Command
{
  SwerveModifier modifier;
  EncoderFollower flFollower,frFollower,blFollower, brFollower;

  public DrivePath() {
    requires(Robot.driveTrain);

    // 3 Waypoints
    Waypoint[] points = new Waypoint[] {
        new Waypoint(0, 0, 0), // Waypoint @ x=-2, y=-2, exit angle=0 radians
        new Waypoint(3, 3, 90) // Waypoint @ x=0, y=0, exit angle=0 radians
    };

    // Create the Trajectory Configuration
    //
    // Arguments:
    // Fit Method: HERMITE_CUBIC or HERMITE_QUINTIC
    // Sample Count: SAMPLES_HIGH (100 000)
    // SAMPLES_LOW (10 000)
    // SAMPLES_FAST (1 000)
    // Time Step: 0.05 Seconds
    // Max Velocity: 1.7 m/s
    // Max Acceleration: 2.0 m/s/s
    // Max Jerk: 60.0 m/s/s/s
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
        0.05, 12, 15, 300);

    // Generate the trajectory
    Trajectory trajectory = Pathfinder.generate(points, config);

    modifier = new SwerveModifier(trajectory).modify(3, 3, SwerveModifier.Mode.SWERVE_DEFAULT);
    
    

    flFollower = new EncoderFollower(modifier.getFrontLeftTrajectory());   // Front Left wheel
    flFollower.configurePIDVA(1.0, 0.0, 0.0, 1.0 / 12, 0);

    frFollower = new EncoderFollower(modifier.getFrontRightTrajectory()); // Front Left wheel
    frFollower.configurePIDVA(1.0, 0.0, 0.0, 1.0 / 12, 0);

    blFollower = new EncoderFollower(modifier.getBackLeftTrajectory()); // Front Left wheel
    blFollower.configurePIDVA(1.0, 0.0, 0.0, 1.0 / 12, 0);

    brFollower = new EncoderFollower(modifier.getBackRightTrajectory()); // Front Left wheel
    brFollower.configurePIDVA(1.0, 0.0, 0.0, 1.0 / 12, 0);

    flFollower.configureEncoder((int) Robot.driveTrain.FLModule.getDriveMotor().getEncoder().getPosition(), 233,
        4.0 / 12.0);
    frFollower.configureEncoder((int) Robot.driveTrain.FLModule.getDriveMotor().getEncoder().getPosition(), 233,
        4.0 / 12.0);
    blFollower.configureEncoder((int) Robot.driveTrain.FLModule.getDriveMotor().getEncoder().getPosition(), 233,
        4.0 / 12.0);
    brFollower.configureEncoder((int) Robot.driveTrain.FLModule.getDriveMotor().getEncoder().getPosition(), 233,
        4.0 / 12.0);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize()
  {
    flFollower.configurePIDVA(RobotMap.FLSwerveDriveGains.p, RobotMap.FLSwerveDriveGains.i, 
        RobotMap.FLSwerveDriveGains.d, 1 / 12, 0);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {


    double flOutput = flFollower.calculate((int)Robot.driveTrain.FLModule.getDriveMotor().getEncoder().getPosition());
    double flDesiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(flFollower.getHeading())); // Bound to -180..180
                                                                                                  // degrees

    double frOutput = flFollower.calculate((int) Robot.driveTrain.FLModule.getDriveMotor().getEncoder().getPosition());
    double frDesiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(flFollower.getHeading())); // Bound to
                                                                                                    // -180..180
    // degrees

    double blOutput = flFollower.calculate((int) Robot.driveTrain.FLModule.getDriveMotor().getEncoder().getPosition());
    double blDesiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(flFollower.getHeading())); // Bound to
                                                                                                    // -180..180
    // degrees

    double brOutput = flFollower.calculate((int) Robot.driveTrain.FLModule.getDriveMotor().getEncoder().getPosition());
    double brDesiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(flFollower.getHeading())); // Bound to
                                                                                                    // -180..180
    // degrees

    Robot.driveTrain.FLModule.setAngle(flDesiredHeading);
    //Robot.driveTrain.FLModule.setSpeed(flOutput);

    Robot.driveTrain.FRModule.setAngle(frDesiredHeading);
    //Robot.driveTrain.FRModule.setSpeed(frOutput);

    Robot.driveTrain.BLModule.setAngle(blDesiredHeading);
    //Robot.driveTrain.BLModule.setSpeed(blOutput);

    Robot.driveTrain.BRModule.setAngle(brDesiredHeading);
    //Robot.driveTrain.BRModule.setSpeed(brOutput);

    //SmartDashboard.putNumber("Motion Profile Expected Distance", flFollower.getSegment().position);
    //SmartDashboard.putNumber("Motion Profile Expected Velocity", flFollower.getSegment().velocity);
    SmartDashboard.putNumber("Motion Profile Expected Heading", flDesiredHeading);
    //SmartDashboard.putNumber("Motion Profile Expected Heading", Robot.driveTrain.FLModule.);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return flFollower.isFinished()&&frFollower.isFinished()&&blFollower.isFinished()&&brFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
    Robot.driveTrain.swerveDrive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
