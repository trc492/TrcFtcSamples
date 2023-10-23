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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcServo;

/**
 * This opmode demonstrates speed controlling a servo arm.
 */
@TeleOp(name="TeleOp: Variable Speed Servo", group="TrcFtcSamples")
@Disabled
public class FtcTeleOpVariableSpeedServo extends FtcOpMode
{
    //
    // Arm constants.
    //
    static final double ARM_MAX_STEPRATE                = 0.2;
    static final double ARM_MIN_POS                     = 0.0;
    static final double ARM_MAX_POS                     = 1.0;

    private FtcDashboard dashboard;
    //
    // Gamepad.
    //
    private FtcGamepad gamepad;
    //
    // Arm subsystem.
    //
    private FtcServo arm;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()
    {

        hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        //
        // Initializing Gamepad.
        //
        gamepad = new FtcGamepad("gamepad", gamepad1);
        gamepad.setYInverted(true);
        //
        // Arm subsystem.
        //
        arm = new FtcServo("arm");
        arm.setStepModeParams(ARM_MAX_STEPRATE, ARM_MIN_POS, ARM_MAX_POS);
        arm.setPosition(ARM_MIN_POS);
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
            //
            // Arm subsystem.
            //
            double armPower = gamepad.getRightStickY(true);
            arm.setPower(armPower);
            dashboard.displayPrintf(2, "Arm:power=%.2f,position=%.2f", armPower, arm.getPosition());
        }
    }   //periodic

}   //class FtcTeleOpVariableSpeedServo
