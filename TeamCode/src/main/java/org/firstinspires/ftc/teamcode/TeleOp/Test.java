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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Controller;


@TeleOp(name="Test", group="Testing")
@Config
public class Test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    private double slowModeModifier = 0.7;

    private DcMotorEx slideLeft1 = null;
    private DcMotorEx slideLeft2 = null;
    private DcMotorEx slideRight = null;
    private DcMotorEx pivot = null;

    // Pivot PID stuff
    double pivotIntegralSum = 0;
    public static double Kp = 0.002;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.2;
    ElapsedTime pivotTimer = new ElapsedTime();
    public static double pivotLastError = 0;

    // Slide PID DOWN
    double downSlideIntegralSum = 0;
    public static double downSlideKp = 0.0001;
    public static double downSlideKi = 0;
    public static double downSlideKd = 0;
    ElapsedTime downSlideTimer = new ElapsedTime();
    public static double downSlideLastError = 0;

    // Slide PID UP
    double upSlideIntegralSum = 0;
    public static double upSlideKp = 0.0001;
    public static double upSlideKi = 0;
    public static double upSlideKd = 0;
    public static double upSlideKf = -0.2;
    ElapsedTime upSlideTimer = new ElapsedTime();
    public static double upSlideLastError = 0;


    public boolean HighGoalState = false;
    public boolean MidGoalState = false;
    public boolean IntakeState = false;
    public boolean InitialState = true;



    private CRServo intake = null;
    private Servo intakeRotate = null;
    private Servo gearshift = null;

    private AnalogInput canandgyro = null;

    private AnalogInput gearpos = null;
    double zeroPoint = 0;

    public static int pivotTarget = 0;

    public static int slidesTarget = 0;


    public static double intakeRotatePos = 0;

    FtcDashboard dashboard = null;
    Telemetry dashboardTelemetry = null;

    Controller driver = null;

    public static double gearShiftTarget = 0;

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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        driver = new Controller(gamepad1);

        gearshift = hardwareMap.get(Servo.class, "gearShift");
        gearpos = hardwareMap.get(AnalogInput.class, "gearpos");
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

        gearshift.setPosition(gearShiftTarget);


        if(driver.dpadUpOnce())
        {
            intakeRotatePos += .05;
        }
        if(driver.dpadDownOnce())
        {
            intakeRotatePos -= .05;
        }

        driver.update();
        intakeRotate.setPosition(intakeRotatePos);

        telemetry.addData("intakeRotatePos", intakeRotatePos);
        telemetry.addData("gearshift", gearpos.getVoltage());

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


}
