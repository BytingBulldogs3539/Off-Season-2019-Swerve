/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.swerve_library.drive.Gains;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap
{

  public static double Robot_W = 1.0;
  public static double Robot_L = 1.0;

  // public static Gains FRSwerveModuleGains = new Gains(0.003, 0.000015, 0.008,
  // 0.0, 0, 0);
  // public static Gains FLSwerveModuleGains = new Gains(0.003, 0.000015, 0.008,
  // 0.0, 0, 0);
  // public static Gains BLSwerveModuleGains = new Gains(0.003, 0.000015, 0.008,
  // 0.0, 0, 0);
  // public static Gains BRSwerveModuleGains = new Gains(0.003, 0.000015, 0.008,
  // 0.0, 0, 0);
  public static Gains FRSwerveModuleGains = new Gains(0.002, 0.00002, 0.00, 0.0, 50, .5, 0);
  public static Gains FLSwerveModuleGains = new Gains(0.002, 0.00002, 0.00, 0.0, 50, .5, 0);
  public static Gains BLSwerveModuleGains = new Gains(0.002, 0.00002, 0.00, 0.0, 50, .5, 0);
  public static Gains BRSwerveModuleGains = new Gains(0.002, 0.00002, 0.00, 0.0, 50, .5, 0);

  public static Gains FRSwerveVelocityGains = new Gains(0.00, 0.000001, 0.00, 0.00018, 100, 1, 0);
  public static Gains FLSwerveVelocityGains = new Gains(0.00, 0.000001, 0.00, 0.00018, 100, 1, 0);
  public static Gains BLSwerveVelocityGains = new Gains(0.00, 0.000001, 0.00, 0.00018, 100, 1, 0);
  public static Gains BRSwerveVelocityGains = new Gains(0.00, 0.000001, 0.00, 0.00018, 100, 1, 0);

  public static Gains FRSwerveDriveGains = new Gains(0.0001, 0, 0.00, 0, 0, 0, 0);
  public static Gains FLSwerveDriveGains = new Gains(0.0001, 0, 0.00, 0, 0, 0, 0);
  public static Gains BLSwerveDriveGains = new Gains(0.0001, 0, 0.00, 0, 0, 0, 0);
  public static Gains BRSwerveDriveGains = new Gains(0.0001, 0, 0.00, 0, 0, 0, 0);

  public static int DriverPort = 0;

  public static int encoderTicksPerRev = 1024;

  public static final double wheelsize = 3.25;

}
