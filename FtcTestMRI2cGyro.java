/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcFtcSamples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcSensor;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcMRI2cGyro;
import TrcFtcLib.ftclib.FtcOpMode;

/**
 * This opmode demonstrates the use of Modern Robotics I2C Gyro Sensor.
 */
@TeleOp(name="Test: Modern Robotics I2C Gyro", group="TrcFtcSamples")
@Disabled
public class FtcTestMRI2cGyro extends FtcOpMode
{
    private FtcDashboard dashboard;
    private FtcMRI2cGyro gyro;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()

    {
        hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTextView(
            ((FtcRobotControllerActivity)hardwareMap.appContext)
                .findViewById(com.qualcomm.ftcrobotcontroller.R.id.textOpMode));
        gyro = new FtcMRI2cGyro("mrGyro");
        gyro.calibrate();
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();
    }   //startMode

    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            final int LABEL_WIDTH = 200;
            dashboard.displayPrintf(1, LABEL_WIDTH, "FirmwareRev: ", "%x", gyro.getFirmwareRevision());
            dashboard.displayPrintf(2, LABEL_WIDTH, "ManufacturerCode: ", "%x", gyro.getManufacturerCode());
            dashboard.displayPrintf(3, LABEL_WIDTH, "IDCode: ", "%x", gyro.getIdCode());
            TrcSensor.SensorData<Double> data = gyro.getHeading();
            //
            // The data may not be ready yet, check it!
            //
            if (data.value != null)
            {
                dashboard.displayPrintf(4, LABEL_WIDTH, "Heading: ", "%.0f", gyro.getHeading().value);
                dashboard.displayPrintf(5, LABEL_WIDTH, "IntegratedZ: ", "%.0f", gyro.getIntegratedZ().value);
                dashboard.displayPrintf(6, LABEL_WIDTH, "RawXYZ: ", "%.0f/%.0f/%.0f",
                                        gyro.getRawX().value, gyro.getRawY().value, gyro.getRawZ().value);
                dashboard.displayPrintf(7, LABEL_WIDTH, "ZOffset: ", "%.0f", gyro.getZOffset().value);
                dashboard.displayPrintf(8, LABEL_WIDTH, "ZScaling: ", "%.0f", gyro.getZScaling().value);
            }
        }
    }   //periodic

}   //class FtcTestMRI2cGyro
