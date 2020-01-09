package frc.robot.swerve_library.drive;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.swerve_library.math.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.ByteEncoder;

/**
 * An implementation of the SwerveEnclosure using neos with spark maxes and
 * encoders
 */
public class MaxSwerveEnclosure extends BaseEnclosure implements SwerveEnclosure
{

    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor;

    private boolean reverseEncoder = false;
    private boolean reverseSteer = false;

    private ByteEncoder steerEncoder;

    private PIDController SteerPidController;

    private CANPIDController drivePidController;

    private Gains steerMotorGains;

    private Gains drivePIDGains;

    public MaxSwerveEnclosure(String name, CANSparkMax driveMotor, CANSparkMax steerMotor, ByteEncoder steerEncoder,
            Gains steerMotorGains, Gains drivePIDGains, double encoderTicksPerRev)
    {

        super(name, encoderTicksPerRev);

        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;
        this.steerMotorGains = steerMotorGains;
        this.drivePIDGains = drivePIDGains;
        this.SteerPidController = new PIDController(this.steerMotorGains.p, this.steerMotorGains.i,
                this.steerMotorGains.d, this.steerMotorGains.f, this.steerMotorGains.iZone, this.steerMotorGains.iMax,
                steerEncoder, steerMotor, 0.005);

        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);

        this.SteerPidController.setOutputRange(-1, 1);
        this.SteerPidController.setAbsoluteTolerance(1);
        this.drivePidController = this.driveMotor.getPIDController();

        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

        // Configure the drivePID Controller.
        drivePidController.setOutputRange(-1, 1);
        drivePidController.setSmartMotionMaxVelocity(4000, 0);
        drivePidController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        drivePidController.setSmartMotionMaxAccel(5000, 0);
        drivePidController.setSmartMotionAllowedClosedLoopError(10, 0);
        drivePidController.setSmartMotionMinOutputVelocity(100, 0);

    }

    @Override
    public void stop()
    {
        this.steerMotor.stopMotor();
        this.driveMotor.stopMotor();

        this.SteerPidController.reset();
        this.SteerPidController.reset();
    }

    @Override
    public void move(double speed, double angle) {
        setAngle(angle);
        setSpeed(speed);
    }

    @Override
    public void setSmartSpeed(double speed)
    {
        this.drivePidController.setP(drivePIDGains.p, 0);
        this.drivePidController.setI(drivePIDGains.i, 0);
        this.drivePidController.setD(drivePIDGains.d, 0);
        this.drivePidController.setFF(drivePIDGains.f, 0);
        this.drivePidController.setIMaxAccum(drivePIDGains.iMax, 0);
        this.drivePidController.setIZone(drivePIDGains.iZone, 0);
        SmartDashboard.putNumber(this.getName() + " Speed Setpoint", speed * 6000);

        drivePidController.setReference(speed * 6000, ControlType.kSmartVelocity);
    }

    @Override
    public void setSpeed(double speed) {
       driveMotor.set(speed);
    }

    @Override
    public void setAngle(double angle)
    {
        SteerPidController.enable();
        //angle /= 360.0;
        SteerPidController.setSetpoint((reverseSteer ? -1 : 1) * angle * encoderTicksPerRev);

    }

    @Override
    public int getEncPosition()
    {
        int reverse = reverseEncoder ? -1 : 1;
        return reverse * steerEncoder.get();
    }

    @Override
    public void setEncPosition(int position)
    {

    }

    public CANSparkMax getDriveMotor()
    {
        return driveMotor;
    }

    public CANSparkMax getSteerMotor()
    {
        return steerMotor;
    }

    public boolean isReverseEncoder()
    {
        return reverseEncoder;
    }

    public void setReverseEncoder(boolean reverseEncoder)
    {
        this.reverseEncoder = reverseEncoder;
    }

    public void setReverseSteerMotor(boolean reverseSteer)
    {
        this.reverseSteer = reverseSteer;
    }

    public PIDController getPIDController()
    {
        return SteerPidController;
    }
}
