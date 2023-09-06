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
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcZXDistanceSensor;

/**
 * This opomode demonstrates the use of the ZX Distance Sensor.
 */
@TeleOp(name="Test: ZX Distance Sensor", group="TrcFtcSamples")
@Disabled
public class FtcTestZXDistanceSensor extends FtcOpMode
{
    private FtcDashboard dashboard;
    private FtcZXDistanceSensor sensor;

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
        sensor = new FtcZXDistanceSensor("zxSensor", FtcZXDistanceSensor.ALTERNATE_I2CADDRESS, false);
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
            TrcSensor.SensorData<Double> data;

            dashboard.displayPrintf(1, LABEL_WIDTH, "Model: ", "%d", sensor.getModelVersion());
            dashboard.displayPrintf(2, LABEL_WIDTH, "RegVersion: ", "%d", sensor.getRegMapVersion());
            dashboard.displayPrintf(3, LABEL_WIDTH, "Status: ", "%02x", sensor.getStatus());
            //
            // The data may not be ready yet, check it!
            //
            TrcSensor.SensorData<FtcZXDistanceSensor.Gesture> gestureData = sensor.getGesture();
            if (gestureData.value != null)
            {
                dashboard.displayPrintf(4, LABEL_WIDTH, "Gesture: ", "%s", gestureData.value.toString());
            }

            data = sensor.getGestureSpeed();
            if (data.value != null)
            {
                dashboard.displayPrintf(5, LABEL_WIDTH, "GestureSpeed: ", "%f", data.value);
            }

            data = sensor.getX();
            if (data.value != null)
            {
                dashboard.displayPrintf(6, LABEL_WIDTH, "X: ", "%f", data.value);
            }

            data = sensor.getZ();
            if (data.value != null)
            {
                dashboard.displayPrintf(7, LABEL_WIDTH, "Z: ", "%f", data.value);
            }

            data = sensor.getLeftRangingData();
            if (data.value != null)
            {
                dashboard.displayPrintf(8, LABEL_WIDTH, "LRng: ", "%f", data.value);
            }

            data = sensor.getRightRangingData();
            if (data.value != null)
            {
                dashboard.displayPrintf(9, LABEL_WIDTH, "RRng: ", "%f", data.value);
            }
        }
    }   //periodic

}   //class FtcTestAndroidSensors
