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
    ElapsedTime pivotTime;
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
    public double PivotUpKp = 0.002;
    public static double PivotUpKf = 0.15;

    public double PivotDownKp = 0.0005;
    public double PivotDownKf = 0.07;


    // Slide PID DOWN
    public static double downSlideKp = 0.0003;

    // Slide PID UP
    public static double upSlideKp = 0.0006;
    public static double upSlideKf = -0.1;


    // States
    public boolean HighGoalState = false;
    public boolean MidGoalState = false;
    public boolean IntakeState = false;
    public boolean InitialState = true;
    public boolean SpecimenState = false;
    public boolean transitionToIntake = true;
    public boolean hangState = false;


    // values
    public static int slidesHigh = -27000;
    public static int pivotScore = 870;
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
    public static double zeroPoint;

    public static int pivotTarget = 0;

    public static int slidesTarget = 0;

    public boolean overrideSlides = false;

    public static double intakeRotatePos = 0;

    FtcDashboard dashboard = null;
    Telemetry dashboardTelemetry = null;

    private boolean isShifting = false;
    public static double neutral = .77;
    public static double lowGear = 0.68;
    public static double highGear =1;


    public static int maxExtension = -22000;

    public double wristError = 0;

    public boolean switchedToHigh = false;
    public boolean switchedToIntake = false;

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
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);


        canandgyro = hardwareMap.get(AnalogInput.class, "canandgyro");

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {


        if (limitSwitch.getState() && !pressedLast && runtime.seconds() > 2)
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
            switchedToHigh = false;
            switchedToIntake = false;
        }
        if(gamepad2.left_bumper && !hangState) // gamepad2 left bumper
        {
            IntakeState = false;
            HighGoalState = false;
            MidGoalState = false;
            InitialState = true;
            transitionToIntake = true;
            SpecimenState = false;
            hangState = false;
            switchedToHigh = false;
            switchedToIntake = false;
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
            switchedToHigh = false;
            switchedToIntake = false;
        }
        if(gamepad2.right_bumper && !hangState) // gamepad2 right bumper
        {
            IntakeState = false;
            HighGoalState = false;
            MidGoalState = true;
            InitialState = false;
            transitionToIntake = true;
            SpecimenState = false;
            hangState = false;
            switchedToHigh = false;
            switchedToIntake = false;
        }
        if(gamepad2.y && !hangState) // specimen: gamepad2 y
        {
            IntakeState = false;
            HighGoalState = false;
            MidGoalState = false;
            InitialState = false;
            transitionToIntake = true;
            SpecimenState = true;
            hangState = false;
            switchedToHigh = false;
            switchedToIntake = false;
        }
        if(gamepad2.a && !hangState) {
            IntakeState = false;
            HighGoalState = false;
            MidGoalState = false;
            InitialState = false;
            transitionToIntake = false;
            SpecimenState = false;
            hangState = true;
            switchedToHigh = false;
            switchedToIntake = false;
        }

        //pivot.setPower(PivotPIDControl(pivotTarget,backRight.getCurrentPosition()));

        if(!hangState && !overrideSlides) {
            if (backRight.getCurrentPosition() > 500)
                setSlidePower(SlideUpPIDControl(slidesTarget, getSlidesPosition()));
            else {
                setSlidePower(SlideDownPIDControl(slidesTarget, getSlidesPosition()));
            }
        }

        if(HighGoalState)
        {
//            if(backRight.getCurrentPosition() < pivotScore)
//            {
//                if(!switchedToHigh)
//                {
//                    pivotTime = new ElapsedTime();
//                    switchedToHigh = true;
//                    distancee = pivotScore - backRight.getCurrentPosition();
//                }
//                double instantTargetPosition = PIDMotionProfile(150,300, distancee, pivotTime.seconds());
//                pivot.setPower(PivotPIDControl(instantTargetPosition,backRight.getCurrentPosition()));
//            }
//            else
//            {
//                pivot.setPower(PivotPIDControl(pivotScore, backRight.getCurrentPosition()));
//            }
            pivotTarget = pivotScore;
            pivot.setPower(PivotUpPIDControl(pivotTarget,backRight.getCurrentPosition()));

            if(backRight.getCurrentPosition() > 740)
            {
                slidesTarget = (int) (63.265*backRight.getCurrentPosition() - 80106.12);
                intakeRotatePos = -0.00306122*backRight.getCurrentPosition() +2.92449;
            }
            else if (backRight.getCurrentPosition() > 500 && backRight.getCurrentPosition() < 740){
                slidesTarget = slidesHigh;
            }
            else
                intakeRotatePos = intakeRotateScore;
            if(gamepad1.right_trigger > 0.1)
                intake.setPower(-1);
            else
                intake.setPower(0);

            if(gamepad2.dpad_up)
            {
                error -= 100;
            }
            else if(gamepad2.dpad_down)
            {
                error += 100;
            }



        }
        if(MidGoalState)
        {
            pivotTarget = pivotScore;
            pivot.setPower(PivotUpPIDControl(pivotTarget,backRight.getCurrentPosition()));
            if(backRight.getCurrentPosition() > 500)
            {
                slidesTarget = -5453;
            }
            intakeRotatePos = intakeRotateScore;
            if(gamepad1.right_trigger > 0.1)
                intake.setPower(-1);
            else
                intake.setPower(0);
        }
        if(IntakeState)
        {
//            if(backRight.getCurrentPosition() > 30)
//            {
//                if(!switchedToIntake)
//                {
//                    pivotTime.reset();
//                    switchedToIntake = true;
//                    distancee = backRight.getCurrentPosition();
//
//                }
//                double instantTargetPosition = PIDMotionProfile(100,300, distancee, pivotTime.seconds());
//
//                pivot.setPower(PivotPIDControl(distancee-instantTargetPosition,backRight.getCurrentPosition()));
//            }
//            else
//                pivot.setPower(PivotPIDControl(0,backRight.getCurrentPosition()));

            pivot.setPower(PivotDownPIDControl(pivotTarget,backRight.getCurrentPosition()));
            pivotTarget = 0;
            if(transitionToIntake) {
                slidesTarget = 0;
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
                intakeRotatePos = (1.184*Math.pow(10,-10))*(Math.pow(getSlidesPosition(),2)) + (7.237*Math.pow(10,-8))*getSlidesPosition()+0.055+wristError;
            }
            else if(gamepad1.x)
            {
                intake.setPower(-1);
            }

            else
            {
                intake.setPower(0);
                intakeRotatePos = 0.25;
            }
            if(slidesTarget < maxExtension)
            {
                slidesTarget = maxExtension;
            }
            if(slidesTarget > 0)
            {
                slidesTarget = 0;
            }

            if(gamepad2.dpad_up)
            {
                wristError += 0.01;
            }
            else if(gamepad2.dpad_down)
            {
                wristError -= 0.01;
            }
            else if(gamepad2.dpad_right)
            {
                wristError = 0;
            }



        }
        if(InitialState)
        {

            pivotTarget = 0;
            pivot.setPower(PivotDownPIDControl(pivotTarget,backRight.getCurrentPosition()));
            slidesTarget = 0;

            if(backRight.getCurrentPosition() > 500)
                intakeRotatePos = 0;
            else
                intakeRotatePos = 1;


            intake.setPower(0);

            if(gamepad2.dpad_down)
            {
                overrideSlides = true;
                setSlidePower(1);
                if(limitSwitch.getState())
                {
                    error = backLeft.getCurrentPosition();
                }
            }
            else {
                overrideSlides = false;
            }

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
            pivotTarget = pivotScore;
            intakeRotatePos = 0;
            gearshift.setPosition(lowGear);
            if(gearpos.getVoltage() > 1.2 && gearpos.getVoltage() < 1.38)
            {
                setSlidePower(-.4);
            }
            else {
                if (gamepad2.right_trigger > 0.1 && getSlidesPosition() < 0) {
                    setSlidePower(-1);
                } else if (gamepad2.left_trigger > 0.1 && getSlidesPosition() > -20000) {
                    setSlidePower(1);
                } else {
                    setSlidePower(0);
                }
            }

            if(slidesTarget < -20000)
                slidesTarget = -20000;
            else if(slidesTarget > 0)
                slidesTarget = 0;

        }

        if(gamepad1.a)
        {
            zeroPoint = canandgyro.getVoltage();
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

        telemetry.addData("slide PID encoder with error",getSlidesPosition());
        telemetry.addData("slide PID encoder",backLeft.getCurrentPosition());
        telemetry.addData("pivot encoder",backRight.getCurrentPosition());
        telemetry.addData("error",error);

        telemetry.addData("voltage canandgyro", canandgyro.getVoltage());

        telemetry.addData("reference",backRight.getCurrentPosition());
        telemetry.addData("target",pivotTarget);

        telemetry.addData("sl1", slideLeft1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("sl2", slideLeft2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("sr", slideRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("pivot", pivot.getCurrent(CurrentUnit.AMPS));

        telemetry.update();
        dashboard.getTelemetry();

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public double PivotDownPIDControl(double reference, double state)
    {
        double error = reference - state;

        double output = (error*PivotDownKp);
        return output+PivotDownKf;

    }

    public double PivotUpPIDControl(double reference, double state)
    {
        double error = reference - state;

        return (error*PivotUpKp) + PivotUpKf;

    }

    public double SlideDownPIDControl(double reference, double state)
    {
        double error = reference - state;

        double output = (error*downSlideKp);
        return output;

    }

    public double SlideUpPIDControl(double reference, double state)
    {
        double error = reference - state;

        double output = (error*upSlideKp);
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

    public double PIDMotionProfile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time,2);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt,2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }

}
