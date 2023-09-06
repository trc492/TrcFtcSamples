/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcOpMode;

/**
 * This opmode demonstrates the use of REV Color Sensor.
 */
@TeleOp(name="Test: REV Color Sensor", group="TrcFtcSamples")
@Disabled
public class FtcTestRevColorSensor extends FtcOpMode
{
    private FtcDashboard dashboard;
    private ColorSensor colorSensor;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()
    {
        //
        // Initializing sensors on or connected to the REV hub.
        //
        dashboard = FtcDashboard.getInstance();
        colorSensor = hardwareMap.get(ColorSensor.class, "colorRangeSensor");
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
            dashboard.displayPrintf(1, "argb=%d/%d/%d/%d",
                                    colorSensor.alpha(), colorSensor.red(), colorSensor.green(), colorSensor.blue());
            int argb = colorSensor.argb();
            dashboard.displayPrintf(2, "colorInt=%d/%d/%d/%d",
                                    (argb >> 24) & 0xff, (argb >> 16) & 0xff, (argb >> 8) & 0xff, argb & 0xff);
            NormalizedRGBA normalizedRGBA = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
            dashboard.displayPrintf(3, "Normalized Color=%f/%f/%f/%f",
                                    normalizedRGBA.alpha, normalizedRGBA.red, normalizedRGBA.blue,
                                    normalizedRGBA.green);
            int normalizedColorInt = normalizedRGBA.toColor();
            dashboard.displayPrintf(4, "Normalized ColorInt=%d/%d/%d/%d",
                                    Color.alpha(normalizedColorInt), Color.red(normalizedColorInt),
                                    Color.green(normalizedColorInt),
                                    Color.blue(normalizedColorInt));
        }
    }   //periodic

}   //class FtcTestRevColorSensor
