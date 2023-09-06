/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcOpMode;

/**
 * This opmode programs the robot to perform line following but not using any PID controlled drive. Instead, it is
 * doing a simple bang-bang control on the robot to wiggle left and right along the edge of the white line on the
 * floor.
 */
@Autonomous(name="Auto: K9Bot Line Following", group="TrcFtcSamples")
@Disabled
public class FtcAutoK9LineFollow extends FtcOpMode
{
    private static final double MOTOR_POWER = 0.15;
    private K9Robot robot;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()
    {
        robot = new K9Robot();
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.startMode();
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.stopMode();
    }   //stopMode

    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        double left;
        double right;
        double lightValue = robot.lightSensor.sensor.getRawLightDetected();
        //
        // Following the left edge of a white line.
        //
        if (lightValue < K9Robot.LIGHT_THRESHOLD)
        {
            //
            // We see the floor, turn right back to the line edge.
            //
            left = MOTOR_POWER;
            right = 0.0;
        }
        else
        {
            //
            // We see the line, turn left back to the line edge.
            //
            left = 0.0;
            right = MOTOR_POWER;
        }
        robot.driveBase.tankDrive(left, right);

        robot.dashboard.displayPrintf(1, "Text: *** Robot Data ***");
        robot.dashboard.displayPrintf(2, "light: %.2f", lightValue);
        robot.dashboard.displayPrintf(3, "left power: %.2f", left);
        robot.dashboard.displayPrintf(4, "right power: %.2f", right);
    }   //periodic

}   //class FtcAutoK9LineFollow
