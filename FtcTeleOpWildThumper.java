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

import java.io.InputStream;

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcSimpleDriveBase;
import TrcCommonLib.trclib.TrcSong;
import TrcCommonLib.trclib.TrcSongPlayer;
import TrcFtcLib.ftclib.FtcAndroidTone;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcMRGyro;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcSongXml;

/**
 * This opmode demonstrates TeleOp control of the Wild Thumper which is a 6-wheel tank robot. It also demonstrates
 * the multi-tasking capability of the TRC Library by playing music while driving the robot.
 */
@TeleOp(name="TeleOp: Wild Thumper", group="TrcFtcSamples")
@Disabled
public class FtcTeleOpWildThumper extends FtcOpMode implements TrcGameController.ButtonHandler
{
    private static final double ATTACK = 0.0;           // in seconds
    private static final double DECAY = 0.0;            // in seconds
    private static final double SUSTAIN = 1.0;          // in proportion
    private static final double RELEASE = 0.02;         // in seconds
    private static final double LOW_BEEP = 440.0;       // in Hz
    private static final double HIGH_BEEP = 880.0;      // in Hz
    private static final double BEEP_DURATION = 0.2;    // in seconds
    private static final double BAR_DURATION = 1.920;   // in seconds

    private static final boolean SIX_WHEELS = false;
    private static final boolean LEFTWHEEL_INVERTED = false;
    private static final boolean RIGHTWHEEL_INVERTED = true;
    private static final boolean BRAKE_MODE_ON = true;

    private FtcDashboard dashboard;
    //
    // Input and sensors.
    //
    private FtcGamepad gamepad;
    private FtcMRGyro gyro;
    //
    // Sound devices.
    //
    private FtcAndroidTone androidTone = null;
    private TrcSong[] collection = null;
    private int songIndex = -1;
    private TrcSongPlayer songPlayer = null;
    private boolean envelopeEnabled = true;
    //
    // Drive Base.
    //
    private TrcSimpleDriveBase driveBase;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()
    {
        hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        //
        // Initializing Gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(true);
        //
        // Initializing sensors.
        //
        gyro = new FtcMRGyro("gyroSensor");
        gyro.calibrate();
        //
        // Initializing Tone support.
        //
        androidTone = new FtcAndroidTone("AndroidTone");
        androidTone.setSoundEnvelope(ATTACK, DECAY, SUSTAIN, RELEASE);
        androidTone.setSoundEnvelopeEnabled(envelopeEnabled);
        int songResourceId = hardwareMap.appContext.getResources().getIdentifier(
            "songcollection", "raw", hardwareMap.appContext.getPackageName());
        InputStream songStream = activity.getResources().openRawResource(songResourceId);
        try
        {
            FtcSongXml songXml = new FtcSongXml("Songs", songStream);
            collection = songXml.getCollection();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
        songIndex = -1;
        //
        // DriveBase subsystem.
        //
        FtcDcMotor lfMotor = new FtcDcMotor("lfWheel");
        FtcDcMotor rfMotor = new FtcDcMotor("rfWheel");
        FtcDcMotor lcMotor = null;
        FtcDcMotor rcMotor = null;
        if (SIX_WHEELS)
        {
            lcMotor = new FtcDcMotor("lcWheel");
            rcMotor = new FtcDcMotor("rcWheel");
        }
        FtcDcMotor lbMotor = new FtcDcMotor("lbWheel");
        FtcDcMotor rbMotor = new FtcDcMotor("rbWheel");

        lfMotor.setMotorInverted(LEFTWHEEL_INVERTED);
        rfMotor.setMotorInverted(RIGHTWHEEL_INVERTED);
        if (SIX_WHEELS)
        {
            lcMotor.setMotorInverted(LEFTWHEEL_INVERTED);
            rcMotor.setMotorInverted(RIGHTWHEEL_INVERTED);
        }
        lbMotor.setMotorInverted(LEFTWHEEL_INVERTED);
        rbMotor.setMotorInverted(RIGHTWHEEL_INVERTED);
        //
        // 6V motors are too fast when driven with 12V so we need to use coast mode or the Thumper will tip forward
        // when stopping.
        //
        lfMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
        rfMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
        if (SIX_WHEELS)
        {
            lcMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
            rcMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
        }
        lbMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
        rbMotor.setBrakeModeEnabled(BRAKE_MODE_ON);

        if (SIX_WHEELS)
        {
            driveBase = new TrcSimpleDriveBase(lfMotor, lcMotor, lbMotor, rfMotor, rcMotor, rbMotor);
        }
        else
        {
            driveBase = new TrcSimpleDriveBase(lfMotor, lbMotor, rfMotor, rbMotor);
        }
    }   //robotInit

    /**
     * This method is called to start/stop the song. It takes care of keeping track of the song state and
     * will do the right thing if it is a start or a resume of the song.
     *
     * @param index specifies the index of the song to start or stop.
     * @param start specifies true to start the song, false to stop.
     */
    private void startSong(int index, boolean start)
    {
        if (start)
        {
            if (songPlayer == null)
            {
                //
                // This is the first time we start the song. So create the song player and associate it with the
                // appropriate tone generator.
                //
                songPlayer = new TrcSongPlayer("SongPlayer", androidTone);
            }
            songPlayer.playSong(collection[index], BAR_DURATION, true, false);
            songIndex = index;
        }
        else if (songPlayer != null)
        {
            //
            // Pause the song.
            //
            songPlayer.pause();
            songIndex = -1;
        }
    }   //startSong

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
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // If there is a SongPlayer, stop it.
        //
        if (songPlayer != null)
        {
            songPlayer.stop();
        }
    }   //stopMode

    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            double left = gamepad.getLeftStickY(true);
            double right = gamepad.getRightStickY(true);
            driveBase.tankDrive(left, right);

            dashboard.displayPrintf(1, "Power(L/R) = %.2f/%.2f", left, right);
            dashboard.displayPrintf(2, "GyroHeading = %.2f", gyro.getZHeading().value);
            dashboard.displayPrintf(3, "SoundEnvelope = %s", envelopeEnabled ? "ON" : "OFF");
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
                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    //
                    // Press this button to turn on/off sound envelope.
                    //
                    if (pressed)
                    {
                        envelopeEnabled = !envelopeEnabled;
                        androidTone.setSoundEnvelopeEnabled(envelopeEnabled);
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    //
                    // Press this button to play a 440Hz beep and release to play a 880Hz beep.
                    //
                    if (pressed)
                    {
                        androidTone.playTone(LOW_BEEP, BEEP_DURATION);
                    }
                    else
                    {
                        androidTone.playTone(HIGH_BEEP, BEEP_DURATION);
                    }
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    if (pressed)
                    {
                        if (songIndex == 0)
                        {
                            //
                            // This song was playing, stop it.
                            //
                            startSong(0, false);
                        }
                        else if (songIndex == 1)
                        {
                            //
                            // The other song was playing, stop that and start this one.
                            //
                            startSong(1, false);
                            startSong(0, true);
                        }
                        else
                        {
                            //
                            // No song was playing, statt this one.
                            //
                            startSong(0, true);
                        }
                    }
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    if (pressed)
                    {
                        if (songIndex == 1)
                        {
                            //
                            // This song was playing, stop it.
                            //
                            startSong(1, false);
                        }
                        else if (songIndex == 0)
                        {
                            //
                            // The other song was playing, stop that and start this one.
                            //
                            startSong(0, false);
                            startSong(1, true);
                        }
                        else
                        {
                            //
                            // No song was playing, statt this one.
                            //
                            startSong(1, true);
                        }
                    }
                    break;
            }
        }
    }   //buttonEvent

}   //class FtcTeleOpWildThumper
