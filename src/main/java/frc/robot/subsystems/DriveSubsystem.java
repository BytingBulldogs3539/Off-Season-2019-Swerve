/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;
import frc.robot.swerve_library.drive.MaxSwerveEnclosure;
import frc.robot.swerve_library.drive.SwerveDrive;
import frc.robot.swerve_library.math.CentricMode;
import frc.robot.utilities.ByteEncoder;

public class DriveSubsystem extends Subsystem
{
  PigeonIMU pigeon = new PigeonIMU(25);

  CANSparkMax FRDrive = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax FRSteer = new CANSparkMax(3, MotorType.kBrushless);
  ByteEncoder FRSteerEncoder = new ByteEncoder(0, 1, 13, false, CounterBase.EncodingType.k1X, 0, false);
  public MaxSwerveEnclosure FRModule = new MaxSwerveEnclosure("FRModule", FRDrive, FRSteer, FRSteerEncoder,
      RobotMap.FRSwerveModuleGains, RobotMap.FRSwerveVelocityGains, RobotMap.encoderTicksPerRev);

  CANSparkMax FLDrive = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax FLSteer = new CANSparkMax(1, MotorType.kBrushless);
  ByteEncoder FLSteerEncoder = new ByteEncoder(2, 3, 11, false, CounterBase.EncodingType.k1X, 0, false);
  public MaxSwerveEnclosure FLModule = new MaxSwerveEnclosure("FLModule", FLDrive, FLSteer, FLSteerEncoder,
      RobotMap.FLSwerveModuleGains, RobotMap.FRSwerveVelocityGains, RobotMap.encoderTicksPerRev);

  CANSparkMax BLDrive = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax BLSteer = new CANSparkMax(8, MotorType.kBrushless);
  ByteEncoder BLSteerEncoder = new ByteEncoder(4, 5, 12, false, CounterBase.EncodingType.k1X, 0, false);
  public MaxSwerveEnclosure BLModule = new MaxSwerveEnclosure("BLModule", BLDrive, BLSteer, BLSteerEncoder,
      RobotMap.BLSwerveModuleGains, RobotMap.FRSwerveVelocityGains, RobotMap.encoderTicksPerRev);

  CANSparkMax BRDrive = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax BRSteer = new CANSparkMax(6, MotorType.kBrushless);
  public ByteEncoder BRSteerEncoder = new ByteEncoder(6, 7, 10, false, CounterBase.EncodingType.k1X, 357.7, true);
  public MaxSwerveEnclosure BRModule = new MaxSwerveEnclosure("BRModule", BRDrive, BRSteer, BRSteerEncoder,
      RobotMap.BRSwerveModuleGains, RobotMap.FRSwerveVelocityGains, RobotMap.encoderTicksPerRev);

  public SwerveDrive swerveDrive = new SwerveDrive(FRModule, FLModule, BLModule, BRModule, RobotMap.Robot_W, RobotMap.Robot_L);  

  public AnalogPotentiometer vision = new AnalogPotentiometer(0);

  public DriveSubsystem()
  {
    pigeon.setYaw(0);

    swerveDrive.setCentricMode(CentricMode.FIELD);

    FRDrive.setInverted(false);
    FLDrive.setInverted(false);
    BLDrive.setInverted(false);
    BRDrive.setInverted(false);

    FRDrive.setIdleMode(IdleMode.kBrake);
    FLDrive.setIdleMode(IdleMode.kBrake);
    BRDrive.setIdleMode(IdleMode.kBrake);
    BLDrive.setIdleMode(IdleMode.kBrake);

    FRModule.setReverseSteerMotor(true);
    FLModule.setReverseSteerMotor(true);
    BLModule.setReverseSteerMotor(true);
    BRModule.setReverseSteerMotor(true);
    FRModule.setReverseEncoder(true);
    FLModule.setReverseEncoder(true);
    BLModule.setReverseEncoder(true);
    BRModule.setReverseEncoder(true);

    FRModule.getPIDController().setName("FR");
    FLModule.getPIDController().setName("FL");
    BRModule.getPIDController().setName("BR");
    BLModule.getPIDController().setName("BL");

    FRSteerEncoder.setName("FR");
    FLSteerEncoder.setName("FL");
    BRSteerEncoder.setName("BR");
    BLSteerEncoder.setName("BL");
  }

  public void drive(double fwd, double str, double rcw)
  {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    swerveDrive.move(fwd, str, rcw, -ypr[0]);
  }

  public void setAllAngles(double angle) {
    FLModule.setAngle(angle);
    FRModule.setAngle(angle);
    BLModule.setAngle(angle);
    BRModule.setAngle(angle);

  }

  public void updateSmartDash()
  {
    SmartDashboard.putNumber("FR Encoder", FRModule.getEncPosition());
    SmartDashboard.putNumber("FL Encoder", FLModule.getEncPosition());
    SmartDashboard.putNumber("BR Encoder", BRModule.getEncPosition());
    SmartDashboard.putNumber("BL Encoder", BLModule.getEncPosition());
    SmartDashboard.putString("Centric Mode", swerveDrive.getCentricMode().toString());

    SmartDashboard.putNumber("FR Velocity", FRDrive.getEncoder().getVelocity());
    SmartDashboard.putNumber("FL Velocity", FLDrive.getEncoder().getVelocity());
    SmartDashboard.putNumber("BL Velocity", BLDrive.getEncoder().getVelocity());
    SmartDashboard.putNumber("BR Velocity", BRDrive.getEncoder().getVelocity());

    
    RobotMap.FLSwerveDriveGains.p = SmartDashboard.getNumber("P Gain",0);
    RobotMap.FLSwerveDriveGains.i = SmartDashboard.getNumber("I Gain", 0);
    RobotMap.FLSwerveDriveGains.d = SmartDashboard.getNumber("D Gain", 0);

    SmartDashboard.putNumber("P Gain", RobotMap.FLSwerveDriveGains.p);
    SmartDashboard.putNumber("I Gain", RobotMap.FLSwerveDriveGains.i);
    SmartDashboard.putNumber("D Gain", RobotMap.FLSwerveDriveGains.d);
    SmartDashboard.putNumber("I Zone", RobotMap.FLSwerveDriveGains.iZone);
    SmartDashboard.putNumber("I Max", RobotMap.FLSwerveDriveGains.iMax);
    SmartDashboard.putNumber("ff", RobotMap.FLSwerveDriveGains.f);

  }

  public void initializeSmartDashBoard()
  {

    SmartDashboard.putData(FRModule.getPIDController());
    SmartDashboard.putData(FLModule.getPIDController());
    SmartDashboard.putData(BLModule.getPIDController());
    SmartDashboard.putData(BRModule.getPIDController());

    SmartDashboard.putNumber("P Gain", RobotMap.FLSwerveDriveGains.p);
    SmartDashboard.putNumber("I Gain", RobotMap.FLSwerveDriveGains.i);
    SmartDashboard.putNumber("D Gain", RobotMap.FLSwerveDriveGains.d);
    SmartDashboard.putNumber("I Zone", RobotMap.FLSwerveDriveGains.iZone);
    SmartDashboard.putNumber("I Max", RobotMap.FLSwerveDriveGains.iMax);
    SmartDashboard.putNumber("ff", RobotMap.FLSwerveDriveGains.f);
  }

  @Override
  public void initDefaultCommand()
  {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveCommand());
  }
}
