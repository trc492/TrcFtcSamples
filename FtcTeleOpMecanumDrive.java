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

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcMecanumDriveBase;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;

/**
 * This opmode demonstrates TeleOp Holonomic Drive on a mecanum drive base robot.
 */
@TeleOp(name="TeleOp: Mecanum Drive", group="TrcFtcSamples")
@Disabled
public class FtcTeleOpMecanumDrive extends FtcOpMode implements TrcGameController.ButtonHandler
{
    private FtcDashboard dashboard;
    private FtcGamepad gamepad;
    private TrcMecanumDriveBase driveBase;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()
    {
        hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        //
        // Initializing sensors.
        //

        //
        // Initializing gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(true);
        //
        // DriveBase subsystem.
        //
        FtcDcMotor lfWheel = new FtcDcMotor("lfWheel");
        FtcDcMotor rfWheel = new FtcDcMotor("rfWheel");
        FtcDcMotor lbWheel = new FtcDcMotor("lbWheel");
        FtcDcMotor rbWheel = new FtcDcMotor("rbWheel");
        rfWheel.setMotorInverted(true);
        rbWheel.setMotorInverted(true);
        driveBase = new TrcMecanumDriveBase(lfWheel, lbWheel, rfWheel, rbWheel, null);
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();
        driveBase.resetOdometry();
    }   //startMode

    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            //
            // DriveBase subsystem.
            //
            double x = gamepad.getLeftStickX(true);
            double y = gamepad.getRightStickY(true);
            double rotation = gamepad.getRightTrigger(true) - gamepad.getLeftTrigger(true);
            driveBase.holonomicDrive(x, y, rotation, false);

            dashboard.displayPrintf(1, "Text: *** Robot Data ***");
            dashboard.displayPrintf(2, "x: %.2f", x);
            dashboard.displayPrintf(3, "y: %.2f", y);
            dashboard.displayPrintf(4, "rotation: %.2f", rotation);
        }
    }   //periodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    break;
            }
        }
    }   //buttonEvent

}   //class FtcTeleOpMecanumDrive
