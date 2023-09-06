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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcRobot;
import TrcFtcLib.ftclib.FtcAnalogInput;
import TrcFtcLib.ftclib.FtcColorSensor;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcDigitalInput;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcImu;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcServo;

/**
 * This opmode demonstrates the use of REV Expansion Hub.
 */
@TeleOp(name="Test: REV Expansion Hub", group="TrcFtcSamples")
@Disabled
public class FtcTestRevHub extends FtcOpMode
{
    private static final int NUM_MOTORS = 4;
    private static final int NUM_SERVOS = 6;
    private static final int NUM_ANALOG_INPUTS = 4;
    private static final int NUM_DIGITAL_INPUTS = 8;

    private FtcGamepad gamepad;
    private TrcGyro gyro;
    private FtcColorSensor colorSensor;
    private final FtcDcMotor[] motors = new FtcDcMotor[NUM_MOTORS];
    private final FtcServo[] servos = new FtcServo[NUM_SERVOS];
    private final FtcAnalogInput[] analogInputs = new FtcAnalogInput[NUM_ANALOG_INPUTS];
    private final FtcDigitalInput[] digitalInputs = new FtcDigitalInput[NUM_DIGITAL_INPUTS];
    private boolean dpadRightPressed = false;

    @Override
    public void robotInit()
    {
        gamepad = new FtcGamepad("GamePad", gamepad1, this::buttonEvent);
        gamepad.setYInverted(true);

        gyro = new FtcImu(
            "imu", RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        colorSensor = new FtcColorSensor("colorSensor");

        for (int i = 0; i < motors.length; i++)
        {
            motors[i] = new FtcDcMotor("motor" + i);
        }

        for (int i = 0; i < servos.length; i++)
        {
            servos[i] = new FtcServo("servo" + i);
        }

        for (int i = 0; i < analogInputs.length; i++)
        {
            analogInputs[i] = new FtcAnalogInput("analog" + i);
        }

        for (int i = 0; i < digitalInputs.length; i++)
        {
            digitalInputs[i] = new FtcDigitalInput("digital" + i);
        }
    }   //robotInit

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        gyro.resetZIntegrator();
        gyro.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        gyro.setEnabled(false);
    }   //stopMode

    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            //
            // Test I2C IMU.
            //
            telemetry.addData("Heading", "Heading: x=%6.1f,y=%6.1f,z=%6.1f",
                              gyro.getXHeading().value, gyro.getYHeading().value, gyro.getZHeading().value);
            telemetry.addData("TurnRate", "TurnRate: x=%6.1f,y=%6.1f,z=%6.1f",
                              gyro.getXRotationRate().value, gyro.getYRotationRate().value,
                              gyro.getZRotationRate().value);
//            telemetry.addData("ImuAngle", "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
//                              getAngularOrientation(
//                                  AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle,
//                              getAngularOrientation(
//                                  AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
//                              getAngularOrientation(
//                                  AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
//            telemetry.addData("Accel", "x=%6.1f,y=%6.1f,z=%6.1f",
//                              imu.accel.getXAcceleration().value, imu.accel.getYAcceleration().value,
//                              imu.accel.getZAcceleration().value);
//            telemetry.addData("Vel", "Vel: x=%6.1f,y=%6.1f,z=%6.1f",
//                              imu.accel.getXVelocity().value, imu.accel.getYVelocity().value,
//                              imu.accel.getZVelocity().value);
//            telemetry.addData("Dist", "x=%6.1f,y=%6.1f,z=%6.1f",
//                              imu.accel.getXDistance().value, imu.accel.getYDistance().value,
//                              imu.accel.getZDistance().value);
            //
            // Test I2C Color Sensor.
            //
            telemetry.addData("ColorRGB", "Color=%x,rgb=%f/%f/%f",
                              colorSensor.getRawData(0, FtcColorSensor.DataType.COLOR_NUMBER).value.intValue(),
                              colorSensor.getRawData(0, FtcColorSensor.DataType.RED).value,
                              colorSensor.getRawData(0, FtcColorSensor.DataType.GREEN).value,
                              colorSensor.getRawData(0, FtcColorSensor.DataType.BLUE).value);
            telemetry.addData("ColorHSV", "HSV=%f/%f/%f",
                              colorSensor.getRawData(0, FtcColorSensor.DataType.HUE).value,
                              colorSensor.getRawData(0, FtcColorSensor.DataType.SATURATION).value,
                              colorSensor.getRawData(0, FtcColorSensor.DataType.VALUE).value);
            //
            // Test DC motors.
            //
            double motorPower = gamepad.getRightStickY(true);
            for (int i = 0; i < motors.length; i++)
            {
                motors[i].setPower(motorPower);
                telemetry.addData("Motor" + i, "power=%.1f, enc=%d", motorPower, motors[i].getPosition());
            }
            //
            // Test Servos.
            //
            double pos = dpadRightPressed ? 1.0 : 0.0;
            for (FtcServo servo : servos)
            {
                servo.setPosition(pos);
            }
            telemetry.addData("Servo", "pos=%.0f", pos);
            //
            // Test Analog Inputs.
            //
            double data;
            for (int i = 0; i < analogInputs.length; i++)
            {
                data = analogInputs[i].getData(0).value;
                telemetry.addData("Analog" + i, "volt=%.2f", data);
            }
            //
            // Test Digital Inputs.
            //
            boolean state;
            for (int i = 0; i < digitalInputs.length; i++)
            {
                state = digitalInputs[i].isActive();
                telemetry.addData("Digital" + i, "state=%s", state);
            }

            telemetry.update();
        }
    }   //periodic

    private void buttonEvent(TrcGameController gameCtrl, int button, boolean pressed)
    {
        switch(button)
        {
            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                dpadRightPressed = pressed;
                break;
        }
    }   //buttonEvent

}   //class FtcTestRevHub
