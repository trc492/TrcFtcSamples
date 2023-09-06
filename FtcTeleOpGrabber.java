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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcServoGrabber;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcServo;

/**
 * This opmode demonstrates TeleOp control of a grabber built with two servo motors.
 */
@TeleOp(name="TeleOp: Dual Servo Grabber", group="TrcFtcSamples")
@Disabled
public class FtcTeleOpGrabber extends FtcOpMode implements TrcGameController.ButtonHandler
{
    //
    // Grabber constants.
    //
    static final double GRABBER_MAX_STEPRATE            = 0.0;
    static final double GRABBER_MIN_POS                 = 0.0;
    static final double GRABBER_MAX_POS                 = 1.0;
    static final double GRABBER_OPEN_POS                = 0.0;
    static final double GRABBER_OPEN_TIME               = 0.5;
    static final double GRABBER_CLOSE_POS               = 1.0;
    static final double GRABBER_CLOSE_TIME              = 0.5;

    private FtcGamepad gamepad;
    private TrcServoGrabber grabber;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()
    {
        final TrcServoGrabber.Parameters grabberParams = new TrcServoGrabber.Parameters()
                .setStepParams(GRABBER_MAX_STEPRATE, GRABBER_MIN_POS, GRABBER_MAX_POS)
                .setServoInverted(true, false)
                .setCloseParams(GRABBER_CLOSE_POS, GRABBER_CLOSE_TIME)
                .setOpenParams(GRABBER_OPEN_POS, GRABBER_OPEN_TIME);

        hardwareMap.logDevices();

        FtcServo leftServo = new FtcServo("grabberLeftServo");
        FtcServo rightServo = new FtcServo("grabberRightServo");
        grabber = new TrcServoGrabber("grabber", leftServo, rightServo, grabberParams, null, null);
        //
        // Initializing gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(true);
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            grabber.setPosition(gamepad.getRightTrigger());
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

}   //class FtcTeleOpGrabber
