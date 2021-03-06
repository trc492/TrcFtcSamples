/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCVDetector;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcOpMode;

/**
 * This opmode demonstrates the use of EasyOpenCV Vision.
 */
@TeleOp(name="Test: EOCV Vision", group="TrcFtcSamples")
@Disabled
public class FtcTestEocvVision extends FtcOpMode
{
    final double HOMOGRAPHY_CAMERA_TOPLEFT_X        = 0.0;
    final double HOMOGRAPHY_CAMERA_TOPLEFT_Y        = 0.0;
    final double HOMOGRAPHY_CAMERA_TOPRIGHT_X       = 639.0;
    final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y       = 0.0;
    final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X     = 0.0;
    final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y     = 479.0;
    final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X    = 639.0;
    final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y    = 479.0;

    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
    // Measurement unit: inches
    final double HOMOGRAPHY_WORLD_TOPLEFT_X         = -22.25;
    final double HOMOGRAPHY_WORLD_TOPLEFT_Y         = 60.0;
    final double HOMOGRAPHY_WORLD_TOPRIGHT_X        = 23.0;
    final double HOMOGRAPHY_WORLD_TOPRIGHT_Y        = 60.0;
    final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X      = -8.75;
    final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y      = 16.0;
    final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X     = 7.5;
    final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y     = 16.0;

    final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
        HOMOGRAPHY_CAMERA_TOPLEFT_X, HOMOGRAPHY_CAMERA_TOPLEFT_Y,
        HOMOGRAPHY_CAMERA_TOPRIGHT_X, HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
        HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
        HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
    final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
        HOMOGRAPHY_WORLD_TOPLEFT_X, HOMOGRAPHY_WORLD_TOPLEFT_Y,
        HOMOGRAPHY_WORLD_TOPRIGHT_X, HOMOGRAPHY_WORLD_TOPRIGHT_Y,
        HOMOGRAPHY_WORLD_BOTTOMLEFT_X, HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
        HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);

    private FtcDashboard dashboard;
    private EocvVision eocvVision;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        //
        // Initialize EOCV Vision.
        //
        int cameraViewId = hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam =
            OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraViewId);
        eocvVision = new EocvVision(
            "EocvVision", 320, 240, cameraRect, worldRect, webcam, OpenCvCameraRotation.UPRIGHT, true, null);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();
        eocvVision.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        eocvVision.setEnabled(false);
    }   //stopMode

    @Override
    public void slowPeriodic(double elapsedTime)
    {
        TrcOpenCVDetector.DetectedObject[] detectedObjects = eocvVision.getDetectedObjects();
        final int maxNumLines = 5;
        int lineIndex = 1;
        int endLine = lineIndex + maxNumLines;

        if (detectedObjects != null)
        {
            int numTargets = Math.min(detectedObjects.length, maxNumLines);

            for (int i = 0; i < numTargets; i++)
            {
                dashboard.displayPrintf(lineIndex, "[%d] %s", i, detectedObjects[i]);
                lineIndex++;
            }
        }

        while (lineIndex < endLine)
        {
            dashboard.displayPrintf(lineIndex, "");
            lineIndex++;
        }
    }   //slowPeriodic

}   //class FtcTestEocvVision