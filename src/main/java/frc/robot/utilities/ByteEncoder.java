/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class ByteEncoder extends Encoder
{
    Counter pwmCounter;
    double offset;
    double buildOffset;

    public ByteEncoder(int dataAPort, int dataBPort, int pwmPort, boolean inverted,
            CounterBase.EncodingType encodingType, double buildOffset, boolean useoffset)
    {

        super(dataAPort, dataBPort, inverted, encodingType);

        pwmCounter = new Counter(pwmPort);
        pwmCounter.setSemiPeriodMode(true); // only count rising edges

        // wait for the pwm signal to be counted
        try
        {
            Thread.sleep(5);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        if (useoffset)
        {

            this.buildOffset = buildOffset;
            offset = getOffset();
            pwmCounter = null;

        }
    }

    public double getOffset()
    {
        // from 1 to 4096 us
        return (((pwmCounter.getPeriod() - 1e-6) / 4095e-6) * 1024) - buildOffset;
    }

    @Override
    public int get()
    {
        return super.get() - (int) offset;
    }

    @Override
    public double pidGet()
    {
        return get();
    }

}
