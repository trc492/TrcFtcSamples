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

import com.qualcomm.robotcore.hardware.IrSeekerSensor;

import TrcCommonLib.trclib.TrcAnalogSensorTrigger;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEnhancedServo;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPidDrive;
import TrcCommonLib.trclib.TrcSensorTrigger;
import TrcCommonLib.trclib.TrcSimpleDriveBase;
import TrcFtcLib.ftclib.FtcBNO055Imu;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcMRColorSensor;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcOpticalDistanceSensor;
import TrcFtcLib.ftclib.FtcServo;

/**
 * This class implements a K9 Robot.
 */
public class K9Robot
{
    static final String moduleName = "K9Robot";
    //
    // NOTE: The following constants must be tuned/updated for your robot in order for it to work properly.
    //
    // PID drive constants.
    //
    static final TrcPidController.PidParameters drivePidParams = new TrcPidController.PidParameters(
        new TrcPidController.PidCoefficients(0.03, 0.0, 0.0), 1.0);
    static final double DRIVE_INCHES_PER_COUNT  = (104.0/7416.5);
    //
    // PID turn constants.
    //
    static final TrcPidController.PidParameters turnPidParams = new TrcPidController.PidParameters(
        new TrcPidController.PidCoefficients(0.05, 0.0, 0.0), 1.0);
    //
    // Velocity PID constants.
    //
    static final double DRIVE_MAX_VELOCITY = 25.0;  // in./sec
    final TrcPidController.PidCoefficients velPidCoeff = new TrcPidController.PidCoefficients(
        0.0, 0.0, 0.0, 1/DRIVE_MAX_VELOCITY);
    //
    // PID line follow constants.
    //
    static final TrcPidController.PidParameters colorPidParams = new TrcPidController.PidParameters(
        new TrcPidController.PidCoefficients(0.1, 0.0, 0.0), 2.0);
    static final double COLOR_BLACK = 0.0;
    static final double COLOR_BLUE = 3.0;
    static final double COLOR_RED = 10.0;
    static final double COLOR_WHITE = 16.0;
    static final double COLOR_DARK_LEVEL = 0.0;
    static final double COLOR_WHITE_LEVEL = 10.0;
    static final double COLOR_LINE_EDGE_LEVEL = ((COLOR_DARK_LEVEL + COLOR_WHITE_LEVEL)/2.0);
    static final double COLOR_LINE_EDGE_DEADBAND = (COLOR_LINE_EDGE_LEVEL*0.25);
    //
    // PID line follow constants.
    //
    static final TrcPidController.PidParameters lightPidParams = new TrcPidController.PidParameters(
        new TrcPidController.PidCoefficients(0.02, 0.0, 0.0), 5.0);

    static final double LIGHT_DARK_LEVEL = 10.0;
    static final double LIGHT_WHITE_LEVEL = 60.0;
    static final double LIGHT_THRESHOLD = ((LIGHT_DARK_LEVEL + LIGHT_WHITE_LEVEL)/2.0);
    //
    // PID IR drive constants.
    //
    static final TrcPidController.PidParameters irDrivePidParams = new TrcPidController.PidParameters(
        new TrcPidController.PidCoefficients(0.8, 0.0, 0.0), 0.1);
    //
    // PID IR turn constants.
    //
    static final TrcPidController.PidParameters irTurnPidParams = new TrcPidController.PidParameters(
        new TrcPidController.PidCoefficients(0.1, 0.0, 0.0), 1.0);
    //
    // Subsystems.
    //
    static final double ARM_RANGE_MIN = 0.2;
    static final double ARM_RANGE_MAX = 0.9;
    static final double CLAW_RANGE_MIN = 0.2;
    static final double CLAW_RANGE_MAX = 0.7;
    static final double SERVO_STEPRATE = 2.0;
    //
    // Global objects.
    //
    public FtcOpMode opMode;
    public FtcDashboard dashboard;
    public TrcDbgTrace globalTracer;
    //
    // Sensors.
    //
    public FtcBNO055Imu imu;
    public TrcGyro gyro;
    public FtcMRColorSensor colorSensor;
    public FtcOpticalDistanceSensor lightSensor;
    public IrSeekerSensor irSeeker;
    public double prevIrAngle = 0.0;
    public double prevIrStrength = 0.0;
    //
    // DriveBase subsystem.
    //
    public FtcDcMotor motorLeft;
    public FtcDcMotor motorRight;
    public TrcSimpleDriveBase driveBase;
    //
    // PID drive.
    //
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroPidCtrl;
    public TrcPidDrive pidDrive;
//    public TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients();
    //
    // PID line follow using color sensor.
    //
    public TrcPidController colorPidCtrl;
    public TrcPidDrive pidLineFollow;
    public TrcAnalogSensorTrigger<FtcMRColorSensor.DataType> colorTrigger;
    //
    // PID line follow using Optical Distance sensor.
    //
    public TrcPidController lightPidCtrl;
    public TrcPidDrive lineFollowDrive;
    public TrcAnalogSensorTrigger<FtcOpticalDistanceSensor.DataType> lightTrigger;
    //
    // PID seek IR.
    //
    public TrcPidController irDrivePidCtrl;
    public TrcPidController irTurnPidCtrl;
    public TrcPidDrive pidSeekIr;
    //
    // Other subsystems.
    //
    public FtcServo armServo;
    public TrcEnhancedServo arm;
    public FtcServo clawServo;
    public TrcEnhancedServo claw;

    public K9Robot()
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        //
        // Initialize sensors.
        //
        imu = new FtcBNO055Imu("imu");
        gyro = imu.gyro;

        colorSensor = new FtcMRColorSensor("colorSensor");
        lightSensor = new FtcOpticalDistanceSensor("light_sensor");
        irSeeker = opMode.hardwareMap.irSeekerSensor.get("irSeeker");
        //
        // DriveBase subsystem.
        //
        motorLeft = new FtcDcMotor("left_wheel");
        motorRight = new FtcDcMotor("right_wheel");
        motorLeft.setOdometryEnabled(true);
        motorRight.setOdometryEnabled(true);
        motorLeft.setInverted(true);
        motorRight.setInverted(false);

        driveBase = new TrcSimpleDriveBase(motorLeft, motorRight, gyro);
        driveBase.setOdometryScales(DRIVE_INCHES_PER_COUNT);
        //
        // PID drive.
        //
        encoderYPidCtrl = new TrcPidController("encoderYPid", drivePidParams, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController("gyroPid", turnPidParams, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, encoderYPidCtrl, gyroPidCtrl);
        //
        // PID line follow using color sensor.
        //
        colorPidCtrl = new TrcPidController("colorPid", colorPidParams, this::getColorValue);
        colorPidCtrl.setAbsoluteSetPoint(true);
        pidLineFollow = new TrcPidDrive("colorLineFollow", driveBase, null, encoderYPidCtrl, colorPidCtrl);
        // In order to line follow, we need to first find the line. We will first use pidDrive to keep the robot
        // moving forward for a set distance. Then colorTrigger will interrupt pidDrive once the line is detected.
        // Then we can use pidLineFollow to follow the line.
        colorTrigger = new TrcAnalogSensorTrigger<>(
                "colorTrigger", colorSensor, 0, FtcMRColorSensor.DataType.WHITE,
                new double[]{COLOR_BLACK, COLOR_WHITE}, this::triggerEvent, true);
        //
        // PID line follow using Optical Distance sensor.
        //
        lightPidCtrl = new TrcPidController("lightPid", lightPidParams, lightSensor.sensor::getRawLightDetected);
        lightPidCtrl.setAbsoluteSetPoint(true);
        lineFollowDrive = new TrcPidDrive(
                "lightLineFollow", driveBase, null, encoderYPidCtrl, lightPidCtrl);
        // In order to line follow, we need to first find the line. We will first use pidDrive to keep the robot
        // moving forward for a set distance. Then lightTrigger will interrupt pidDrive once the line is detected.
        // Then we can use lineFollowDrive to follow the line.
        lightTrigger = new TrcAnalogSensorTrigger<>(
                "lightTrigger", lightSensor, 0, FtcOpticalDistanceSensor.DataType.RAW_LIGHT_DETECTED,
                new double[]{LIGHT_DARK_LEVEL, LIGHT_WHITE_LEVEL}, this::triggerEvent, true);
        //
        // PID IR seeking.
        //
        irDrivePidCtrl = new TrcPidController("irDrivePid", irDrivePidParams, this::getIrStrength);
        irDrivePidCtrl.setAbsoluteSetPoint(true);
        irTurnPidCtrl = new TrcPidController("irTurnPid", irTurnPidParams, this::getIrAngle);
        irDrivePidCtrl.setAbsoluteSetPoint(true);
        pidSeekIr = new TrcPidDrive("seekIr", driveBase, null, irDrivePidCtrl, irTurnPidCtrl);
        //
        // Arm subsystem.
        //
        armServo = new FtcServo("servo_1");
        armServo.setLogicalRange(ARM_RANGE_MIN, ARM_RANGE_MAX);
        arm = new TrcEnhancedServo("arm", armServo);
        arm.setPosition(ARM_RANGE_MIN);
        //
        // Claw subsystem.
        //
        clawServo = new FtcServo("servo_6");
        clawServo.setLogicalRange(CLAW_RANGE_MIN, CLAW_RANGE_MAX);
        claw = new TrcEnhancedServo("claw", clawServo);
        claw.setPosition(CLAW_RANGE_MIN);
    }   //K9Robot

    /**
     * This method is called once right before the opmode starts (i.e. at the time the "Play" button on the DS is
     * pressed).
     */
    public void startMode()
    {
        dashboard.clearDisplay();
        gyro.setEnabled(true);
        driveBase.setOdometryEnabled(true);
        colorSensor.sensor.enableLed(true);
    }   //startMode

    /**
     * This method is called once before the opmode is exiting.
     */
    public void stopMode()
    {
        gyro.setEnabled(false);
        driveBase.setOdometryEnabled(false);
        colorSensor.sensor.enableLed(false);
    }   //stopMode

    /**
     * This method is called when a threshold has been crossed.
     *
     * @param currZone specifies the zone it is going into.
     * @param prevZone specifies the zone it is coming out of.
     * @param zoneValue specifies the actual sensor value.
     */
    private void triggerEvent(int currZone, int prevZone, double zoneValue)
    {
        if (pidDrive.isActive() && currZone > 0)
        {
            pidDrive.cancel();
        }
    }   //triggerEvent

    /**
     * This method reads and returns the color sensor value.
     *
     * @return color sensor value.
     */
    private double getColorValue()
    {
        double input = colorSensor.sensor.alpha();
        //
        // Give it a deadband to minimize fish tailing.
        //
        if (Math.abs(input - COLOR_LINE_EDGE_LEVEL) < COLOR_LINE_EDGE_DEADBAND)
        {
            input = COLOR_LINE_EDGE_LEVEL;
        }

        return input;
    }   //getColorValue

    /**
     * This method reads and returns the IR strength value.
     *
     * @return IR strength value.
     */
    private double getIrStrength()
    {
        double input;
        //
        // Get the IR strength.
        //
        if (irSeeker.signalDetected())
        {
            input = irSeeker.getStrength();
            prevIrStrength = input;
        }
        else
        {
            input = prevIrStrength;
        }

        return input;
    }   //getIrStrength

    /**
     * This method reads and returns the IR angle value.
     *
     * @return IR angle value.
     */
    private double getIrAngle()
    {
        double input;
        //
        // Get the IR direction.
        //
        if (irSeeker.signalDetected())
        {
            input = irSeeker.getAngle();
            prevIrAngle = input;
        }
        else
        {
            input = prevIrAngle;
        }

        return input;
    }   //getIrAngle

    /**
     * This method prints the robot information of the current state in the state machine.
     *
     * @param elapsedTime specifies the program elapsed time.
     * @param stateName specifies the state machine state.
     * @param yTarget specifies the drive base target Y position.
     * @param headingTarget specifies the drive base target heading.
     */
    public void traceStateInfo(double elapsedTime, String stateName, double yTarget, double headingTarget)
    {
        globalTracer.traceInfo(
            moduleName, "[%5.3f] >>>>> %s: yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f",
            elapsedTime, stateName, driveBase.getYPosition(), yTarget, driveBase.getHeading(), headingTarget);
    }   //traceStateInfo

}   //class K9Robot
