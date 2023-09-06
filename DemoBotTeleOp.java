/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import TrcCommonLib.trclib.TrcDriveBase;
import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcMecanumDriveBase;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcSimpleDriveBase;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcDigitalInput;
import TrcFtcLib.ftclib.FtcGamepad;
import TrcFtcLib.ftclib.FtcImu;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcServo;

/**
 * This class implements two demo bots: a mecanum bot and a tank bot.
 */
public class DemoBotTeleOp extends FtcOpMode
{
    //
    // Hardware name constants.
    //
    private static final String HWNAME_IMU                      = "imu";
    private static final String HWNAME_LFWHEEL                  = "lfWheel";
    private static final String HWNAME_RFWHEEL                  = "rfWheel";
    private static final String HWNAME_LBWHEEL                  = "lbWheel";
    private static final String HWNAME_RBWHEEL                  = "rbWheel";
    private static final String HWNAME_ELEVATOR_MOTOR           = "elevatorMotor";
    private static final String HWNAME_ELEVATOR_LOWER_LIMIT     = "elevatorLowerLimit";
    private static final String HWNAME_ELEVATOR_UPPER_LIMIT     = "elevatorUpperLimit";
    private static final String HWNAME_SHOOTER_GATE             = "shooterGate";
    //
    // Drive Base constants.
    //
    private static final double MECANUM_BOT_INCHES_PER_COUNT    = 0.01262613869854569477726554746663;
    private static final double TANK_BOT_INCHES_PER_COUNT       = (104.0/7416.5);
    //
    // Elevator subsystem constants.
    //
    private static final double ELEVATOR_INCHES_PER_COUNT       = 5.625/8498;
    private static final double ELEVATOR_ZERO_OFFSET            = 16.25;
    private static final double ELEVATOR_MIN_HEIGHT             = ELEVATOR_ZERO_OFFSET - 0.1;
    private static final double ELEVATOR_MAX_HEIGHT             = 23.0;
    private static final double ELEVATOR_CAL_POWER              = -0.3;
    //
    // Shooter Gate.
    //
    private static final double SHOOTER_GATE_OPEN_POS           = 1.0;
    private static final double SHOOTER_GATE_CLOSE_POS          = 0.0;
    //
    // TeleOp constants.
    //
    private static final double FAST_DRIVE_SCALE                = 1.0;
    private static final double SLOW_DRIVE_SCALE                = 0.5;
    //
    // Global objects.
    //
    private final boolean isMecanumBot;
    private final boolean hasElevator;
    private final boolean hasShooterGate;
    private FtcDashboard dashboard;
    //
    // Sensors.
    //
    private TrcGyro gyro;
    //
    // DriveBase subsystem.
    //
    private TrcDriveBase driveBase;
    //
    // Elevator.
    //
    private FtcDcMotor elevator;
    //
    // Shooter gate.
    //
    private FtcServo shooterGate;
    //
    // TeleOp.
    //
    private FtcGamepad gamepad;
    private double drivePowerScale = FAST_DRIVE_SCALE;
    private boolean manualOverride = false;
    private boolean fieldMode = false;

    public DemoBotTeleOp(boolean isMecanumBot, boolean hasElevator, boolean hasShooterGate)
    {
        this.isMecanumBot = isMecanumBot;
        this.hasElevator = hasElevator;
        this.hasShooterGate = hasShooterGate;
    }   //DemoBotTeleOp

    //
    // Implements FtcOpMode abstract methods.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station phone is pressed.
     */
    @Override
    public void robotInit()
    {
        //
        // Initialize global objects.
        //
        FtcOpMode opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = FtcDashboard.getInstance();
        //
        // Initialize sensors.
        //
        gyro = new FtcImu(
            HWNAME_IMU, RevHubOrientationOnRobot.LogoFacingDirection.UP,
            isMecanumBot? RevHubOrientationOnRobot.UsbFacingDirection.RIGHT:
                          RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        //
        // DriveBase subsystem.
        //
        FtcDcMotor lfWheel = new FtcDcMotor(HWNAME_LFWHEEL);
        FtcDcMotor rfWheel = new FtcDcMotor(HWNAME_RFWHEEL);
        FtcDcMotor lbWheel = new FtcDcMotor(HWNAME_LBWHEEL);
        FtcDcMotor rbWheel = new FtcDcMotor(HWNAME_RBWHEEL);
        if (isMecanumBot)
        {
            rfWheel.setMotorInverted(true);
            rbWheel.setMotorInverted(true);
            driveBase = new TrcMecanumDriveBase(lfWheel, lbWheel, rfWheel, rbWheel, gyro);
            driveBase.setOdometryScales(MECANUM_BOT_INCHES_PER_COUNT);
        }
        else
        {
            rfWheel.setMotorInverted(true);
            rbWheel.setMotorInverted(true);
            driveBase = new TrcSimpleDriveBase(lfWheel, lbWheel, rfWheel, rbWheel, gyro);
            driveBase.setOdometryScales(TANK_BOT_INCHES_PER_COUNT);
        }

        if (hasElevator)
        {
            FtcDigitalInput lowerLimitSwitch = new FtcDigitalInput(HWNAME_ELEVATOR_LOWER_LIMIT);
            FtcDigitalInput upperLimitSwitch = new FtcDigitalInput(HWNAME_ELEVATOR_UPPER_LIMIT);
            lowerLimitSwitch.setInverted(true);
            upperLimitSwitch.setInverted(true);
            elevator = new FtcDcMotor(HWNAME_ELEVATOR_MOTOR, lowerLimitSwitch, upperLimitSwitch, null);
            elevator.setPositionSensorScaleAndOffset(ELEVATOR_INCHES_PER_COUNT, ELEVATOR_ZERO_OFFSET);
        }

        if (hasShooterGate)
        {
            shooterGate = new FtcServo(HWNAME_SHOOTER_GATE);
        }
        //
        // Initializing gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this::buttonEvent);
        gamepad.setYInverted(true);
    }   //robotInit

    //
    // Implements TrcRobot.RobotMode interface.
    //

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station phone is pressed. Typically, you put code that will prepare the robot for
     * start of competition here such as resetting the encoders/sensors and enabling some sensors to start
     * sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();
        gyro.setEnabled(true);
        driveBase.resetOdometry();
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        gyro.setEnabled(false);
        driveBase.setOdometryEnabled(false);
    }   //stopMode


    /**
     * This method is called periodically at a fast rate. Typically, you put code that requires servicing at a
     * high frequency here. To make the robot as responsive and as accurate as possible especially in autonomous
     * mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            //
            // DriveBase subsystem.
            //
            if (driveBase instanceof TrcMecanumDriveBase)
            {
                double xPower = gamepad.getLeftStickX(true) * drivePowerScale;
                double yPower = gamepad.getLeftStickY(true) * drivePowerScale;
                double rotPower = gamepad.getRightStickX(true) * drivePowerScale;

                driveBase.holonomicDrive(null, xPower, yPower, rotPower, fieldMode? gyro.getZHeading().value: 0.0);
                dashboard.displayPrintf(1, "Mecanum: x=%.2f, y=%.2f, rot=%.2f", xPower, yPower, rotPower);
            }
            else
            {
                double leftPower = gamepad.getLeftStickY(true) * drivePowerScale;
                double rightPower = gamepad.getRightStickY(true) * drivePowerScale;

                driveBase.tankDrive(leftPower, rightPower);
                dashboard.displayPrintf(1, "WestCoast: left=%.2f, right=%.2f", leftPower, rightPower);
            }
            dashboard.displayPrintf(2, "%s: heading=%.3f", fieldMode? "Field": "Robot", gyro.getZHeading().value);
            //
            // Elevator subsystem.
            //
            if (elevator != null)
            {
                double elevatorPower = gamepad.getRightTrigger(true) - gamepad.getLeftTrigger(true);

                if (manualOverride)
                {
                    elevator.setPower(elevatorPower);
                }
                else
                {
                    elevator.setPidPower(elevatorPower, ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT, true);
                }
                dashboard.displayPrintf(
                    3, "Elevator: power=%.2f, pos=%.2f, limitSw(lower/upper)=%s/%s",
                    elevatorPower, elevator.getPosition(),
                    elevator.isLowerLimitSwitchActive(), elevator.isUpperLimitSwitchActive());
            }
        }
    }   //periodic

    /**
     * This method is called when a game controller button has been pressed or released.
     *
     * @param gamepad specifies the gamepad that generated the event.
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false if released.
     */
    private void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (driveBase instanceof TrcMecanumDriveBase && pressed)
                    {
                        fieldMode = !fieldMode;
                        if (fieldMode)
                        {
                            gyro.resetZIntegrator();
                        }
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (shooterGate != null)
                    {
                        if (pressed)
                        {
                            shooterGate.setPosition(SHOOTER_GATE_OPEN_POS);
                        }
                        else
                        {
                            shooterGate.setPosition(SHOOTER_GATE_CLOSE_POS);
                        }
                    }
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    drivePowerScale = pressed? SLOW_DRIVE_SCALE: FAST_DRIVE_SCALE;
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    manualOverride = pressed;
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    if (elevator != null && pressed)
                    {
                        elevator.zeroCalibrate(ELEVATOR_CAL_POWER);
                    }
                    break;
            }
        }
    }   //buttonEvent

}   //class DemoBotTeleOp
