/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="FirstTeleOp", group="Testing")
@Config
public class FirstTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    private double slowModeModifier = 0.35;

    private DcMotorEx slideLeft1 = null;
    private DcMotorEx slideLeft2 = null;
    private DcMotorEx slideRight = null;
    private DcMotorEx pivot = null;

    private DigitalChannel limitSwitch = null;
    // Pivot PID stuff
    double pivotIntegralSum = 0;
    public double Kp = 0.002;
    public double Ki = 0;
    public double Kd = 0;
    public double Kf = 0.2;
    ElapsedTime pivotTimer = new ElapsedTime();
    public double pivotLastError = 0;

    // Slide PID DOWN
    double downSlideIntegralSum = 0;
    public double downSlideKp = 0.0001;
    public double downSlideKi = 0;
    public double downSlideKd = 0;
    ElapsedTime downSlideTimer = new ElapsedTime();
    public double downSlideLastError = 0;

    // Slide PID UP
    double upSlideIntegralSum = 0;
    public double upSlideKp = 0.0001;
    public double upSlideKi = 0;
    public double upSlideKd = 0;
    public double upSlideKf = -0.2;
    ElapsedTime upSlideTimer = new ElapsedTime();
    public double upSlideLastError = 0;


    // States
    public boolean HighGoalState = false;
    public boolean MidGoalState = false;
    public boolean IntakeState = false;
    public boolean InitialState = true;
    public boolean SpecimenState = false;
    public boolean transitionToIntake = true;
    public boolean hangState = false;

    // values
    public static int slidesHigh = -26000;
    public static int pivotScore = 800;
    public static double intakeRotateScore = 0.35;

    public static int pivotSpecimen = 500;
    public static double rotatePosSpecimen = 0.3;
    public static int slidesTargetSpecimen = -8000;
    public static int slidesTargetSpecimenAfter = -15000;


    private int error = 0;
    private boolean pressedLast = false;

    private CRServo intake = null;
    private Servo intakeRotate = null;
    private Servo gearshift = null;
    private AnalogInput gearpos = null;

    private AnalogInput canandgyro = null;
    double zeroPoint = 0;

    public static int pivotTarget = 0;

    public static int slidesTarget = 0;


    public static double intakeRotatePos = 0;

    FtcDashboard dashboard = null;
    Telemetry dashboardTelemetry = null;

    private boolean isShifting = false;
    public static double neutral = .77;
    public static double lowGear = 0.68;
    public static double highGear =1;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(20);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight  = hardwareMap.get(DcMotor.class, "fr");
        backLeft  = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft1 = hardwareMap.get(DcMotorEx.class, "l1");
        slideLeft2 = hardwareMap.get(DcMotorEx.class, "l2");
        slideRight = hardwareMap.get(DcMotorEx.class, "r1");

        slideLeft1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot  = hardwareMap.get(DcMotorEx.class, "p");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);


        canandgyro = hardwareMap.get(AnalogInput.class, "canandgyro");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        zeroPoint = canandgyro.getVoltage();

        intake = hardwareMap.get(CRServo.class, "intake");
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        gearshift = hardwareMap.get(Servo.class, "gearShift");

        gearpos = hardwareMap.get(AnalogInput.class, "gearpos");


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start()
    {
        runtime.reset();
        pivotTimer.reset();
        downSlideTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {


        if (limitSwitch.getState() && !pressedLast)
        {
            error = backLeft.getCurrentPosition();
            pressedLast = true;
        }
        if(!limitSwitch.getState())
        {
            pressedLast = false;
        }


        if((gamepad2.left_trigger > .1) && !hangState) // gamepad2 left trigger
        {
            IntakeState = true;
            HighGoalState = false;
            MidGoalState = false;
            InitialState = false;
            SpecimenState = false;
            hangState = false;
        }
        if(gamepad2.left_bumper) // gamepad2 left bumper
        {
            IntakeState = false;
            HighGoalState = false;
            MidGoalState = false;
            InitialState = true;
            transitionToIntake = true;
            SpecimenState = false;
            hangState = false;
        }
        if(gamepad2.right_trigger > 0.1 && !hangState) // gamepad2 right trigger one press
        {
            IntakeState = false;
            HighGoalState = true;
            MidGoalState = false;
            InitialState = false;
            transitionToIntake = true;
            SpecimenState = false;
            hangState = false;
        }
        if(gamepad2.right_bumper) // gamepad2 right bumper
        {
            IntakeState = false;
            HighGoalState = false;
            MidGoalState = true;
            InitialState = false;
            transitionToIntake = true;
            SpecimenState = false;
            hangState = false;
        }
        if(gamepad2.y) // specimen: gamepad2 y
        {
            IntakeState = false;
            HighGoalState = false;
            MidGoalState = false;
            InitialState = false;
            transitionToIntake = true;
            SpecimenState = true;
            hangState = false;

        }
        if(gamepad2.a) {
            IntakeState = false;
            HighGoalState = false;
            MidGoalState = false;
            InitialState = false;
            transitionToIntake = false;
            SpecimenState = false;
            hangState = true;
        }

        pivot.setPower(PivotPIDControl(pivotTarget,backRight.getCurrentPosition()));

        if(!hangState) {
            if (backRight.getCurrentPosition() > 500)
                setSlidePower(SlideUpPIDControl(slidesTarget, getSlidesPosition()));
            else
                setSlidePower(SlideDownPIDControl(slidesTarget, getSlidesPosition()));
        }

        if(HighGoalState)
        {
            pivotTarget = pivotScore;
            if(backRight.getCurrentPosition() > 500)
            {
                slidesTarget = slidesHigh;
            }
            intakeRotatePos = intakeRotateScore;
            if(gamepad1.right_trigger > 0.1)
                intake.setPower(-1);
            else
                intake.setPower(0);

        }
        if(MidGoalState)
        {
            pivotTarget = pivotScore;
            if(backRight.getCurrentPosition() > 500)
            {
                slidesTarget = -3453;
            }
            intakeRotatePos = intakeRotateScore;
            if(gamepad1.right_trigger > 0.1)
                intake.setPower(-1);
            else
                intake.setPower(0);
        }
        if(IntakeState)
        {
            pivotTarget = 0;
            if(transitionToIntake) {
                slidesTarget = -1000;
                transitionToIntake = false;
            }
            // right trigger extend, left trigger retract
            // right button intake
            if(gamepad1.left_bumper) {
                if (gamepad1.right_trigger > 0.1) {
                    slidesTarget -= 600;
                } else if (gamepad1.left_trigger > 0.1) {
                    slidesTarget += 600;
                }
            } else
            {
                if (gamepad1.right_trigger > 0.1) {
                    slidesTarget -= 1000;
                } else if (gamepad1.left_trigger > 0.1) {
                    slidesTarget += 1000;
                }
            }
            if(gamepad1.right_bumper)
            {
                intake.setPower(1);
                intakeRotatePos = (1.184*Math.pow(10,-10))*(Math.pow(getSlidesPosition(),2)) + (7.237*Math.pow(10,-8))*getSlidesPosition()+0.055;
            }
            else
            {
                intake.setPower(0);
                intakeRotatePos = 0.25;
            }
            if(slidesTarget < -18500)
            {
                slidesTarget = -18500;
            }
            if(slidesTarget > 0)
            {
                slidesTarget = 0;
            }

        }
        if(InitialState)
        {
            pivotTarget = 0;
            slidesTarget = 0;
            if(backRight.getCurrentPosition() < 500)
                intakeRotatePos = 1;
            intake.setPower(0);
        }

        if(SpecimenState)
        {
            intakeRotatePos = rotatePosSpecimen;
            pivotTarget = pivotSpecimen;
            if(gamepad1.right_trigger > 0.1)
            {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }
            if(gamepad1.left_trigger > 0.1)
            {
                slidesTarget = slidesTargetSpecimenAfter;
            }
            else {
                slidesTarget = slidesTargetSpecimen;
            }
        }
        if(hangState)
        {
            gearshift.setPosition(lowGear);
            if(gearpos.getVoltage() > 1.2 && gearpos.getVoltage() < 1.38)
            {
                setSlidePower(-.4);
            }
            else {
                if (gamepad2.right_trigger > 0.1) {
                    setSlidePower(-1);
                } else if (gamepad2.left_trigger > 0.1) {
                    setSlidePower(1);
                } else {
                    setSlidePower(0);
                }
            }

        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = 2*AngleUnit.normalizeRadians(canandgyro.getVoltage() - zeroPoint);;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if(gamepad1.left_bumper) {
            frontLeft.setPower(frontLeftPower*slowModeModifier);
            backLeft.setPower(backLeftPower*slowModeModifier);
            frontRight.setPower(frontRightPower*slowModeModifier);
            backRight.setPower(backRightPower*slowModeModifier);
        }
        else {
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }

        intakeRotate.setPosition(intakeRotatePos);
        telemetry.addData("intakeArm",intakeRotatePos);


        telemetry.addData("slide PID encoder",getSlidesPosition());
        telemetry.addData("pivot encoder",backRight.getCurrentPosition());
        telemetry.addData("error",error);

        telemetry.update();
        dashboard.getTelemetry();

        dashboardTelemetry.addData("error",error);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public double PivotPIDControl(double reference, double state)
    {
        double error = reference - state;
        pivotIntegralSum += error * pivotTimer.seconds();
        double derivative = (error - pivotLastError) / pivotTimer.seconds();
        pivotLastError = error;

        pivotTimer.reset();

        double output = (error*Kp) + (derivative*Kd) + (pivotIntegralSum*Ki);
        return output+Kf;

    }

    public double SlideDownPIDControl(double reference, double state)
    {
        double error = reference - state;
        downSlideIntegralSum += error * downSlideTimer.seconds();
        double derivative = (error - downSlideLastError) / downSlideTimer.seconds();
        downSlideLastError = error;

        downSlideTimer.reset();

        double output = (error*downSlideKp) + (derivative*downSlideKd) + (downSlideIntegralSum*downSlideKi);
        return output;

    }

    public double SlideUpPIDControl(double reference, double state)
    {
        double error = reference - state;
        upSlideIntegralSum += error * upSlideTimer.seconds();
        double derivative = (error - upSlideLastError) / upSlideTimer.seconds();
        upSlideLastError = error;

        upSlideTimer.reset();

        double output = (error*upSlideKp) + (derivative*upSlideKd) + (upSlideIntegralSum*upSlideKi);
        return output+upSlideKf;

    }

    public void setSlidePower(double power)
    {
        slideLeft1.setPower(power);
        slideLeft2.setPower(power);
        slideRight.setPower(power);

    }

    public int getSlidesPosition()
    {
        return backLeft.getCurrentPosition() - error;
    }

}
