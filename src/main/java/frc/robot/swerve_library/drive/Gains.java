/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.swerve_library.drive;

/**
 * Encapsulates PIDF and related variables
 */
public class Gains
{
    public double p;
    public double i;
    public double d;
    public double f;
    public double iZone;
    public double iMax;
    public double peakOutput;

    /**
     * Allows us to hold all of the PID values in one spot and not have a whole
     * bunch in robotmap.
     * 
     * For more infomation on how the PIDF values work.
     * https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html?highlight=izone#closed-loop-configs-per-slot-four-slots-available
     * 
     * @param P
     *                       The P value.
     * @param I
     *                       The I value.
     * @param D
     *                       The D value.
     * @param F
     *                       The F value.
     * @param iZone
     *                       Integral Zone can be used to auto clear the integral
     *                       accumulator if the sensor pos is too far from the
     *                       target. This prevent unstable oscillation if the kI is
     *                       too large. Value is in sensor units.
     * @param peakOutput
     *                       The max output that the pid can reach (a value of 1
     *                       represents full backward and forward for the max
     *                       output).
     */
    public Gains(double p, double i, double d, double f, double iZone, double iMax, double peakOutput)
    {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.iZone = iZone;
        this.iMax = iMax;
        this.peakOutput = peakOutput;

    }
}