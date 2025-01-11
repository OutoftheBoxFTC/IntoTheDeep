package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.FirstTeleOp;

import java.util.Arrays;

@Config
@Autonomous(name = "SamplePreloadAuto", group = "Autonomous")
public class SamplePreloadAuto extends LinearOpMode {

    public DcMotorEx backLeft;
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ScoreSystem robot = new ScoreSystem(hardwareMap);

        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Pose2d currentPose = null;

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50),
                new AngularVelConstraint(2*Math.PI)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-40.0, 35.0);

        TrajectoryActionBuilder one = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-16.26, 0))
                .strafeToLinearHeading(new Vector2d(-11.22, -40.92),Math.toRadians(130))                ;

        TrajectoryActionBuilder twohalf = one.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-24.3088,-18.32), Math.toRadians(-126))

                //.strafeToSplineHeading(new Vector2d(-28.177, -19.79), Math.toRadians(-110), baseVelConstraint, baseAccelConstraint) // move forward to first sample pickup
                ;


        TrajectoryActionBuilder three = twohalf.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-16.3088,-40.92), Math.toRadians(130))
                .strafeToLinearHeading(new Vector2d(-11.22, -40.92),Math.toRadians(130)) // deliver first sample (previous: -12.53, -42.74
                ;

        TrajectoryActionBuilder four = three.endTrajectory().fresh()
                //.strafeToLinearHeading(new Vector2d(-35.40,-30.61), Math.toRadians(-91), baseVelConstraint, baseAccelConstraint) // pickup second
                .strafeToSplineHeading(new Vector2d(-25.8088,-29.92), Math.toRadians(-126))
                ;

        TrajectoryActionBuilder five = four.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-11.22, -40.92),Math.toRadians(130)) // deliver second sample
                ;

        TrajectoryActionBuilder six = five.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-28.8088, -40.7), Math.toRadians(-126)) // pickup third
                ;

        TrajectoryActionBuilder seven = six.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-11.22, -40.92),Math.toRadians(130)) // deliver third sample
                ;

//        TrajectoryActionBuilder eight = seven.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-50, -23.73),Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(-50, -14.25),Math.toRadians(90))
//                ;

        TrajectoryActionBuilder eight = seven.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-30.53, -42.74),Math.toRadians(130))
                .strafeToLinearHeading(new Vector2d(-49.327, -14.976), Math.toRadians(113))
                ;

        TrajectoryActionBuilder nine = eight.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-48, -14.9), Math.toRadians(80))
                ;

        TrajectoryActionBuilder ten = nine.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50, -20.25),Math.toRadians(80))
                .strafeToLinearHeading(new Vector2d(-12.53, -42.74),Math.toRadians(130)) // deliver fourth sample
                ;





        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        Action trajOne = one.build();
        Action trajTwoHalf = twohalf.build();
        Action trajThree = three.build();
        Action trajFour = four.build();
        Action trajFive = five.build();
        Action trajSix = six.build();
        Action trajSeven = seven.build();
        Action trajEight = eight.build();
        Action trajNine = nine.build();
        Action trajTen = ten.build();

        telemetry.update();
        waitForStart();


        drive.updatePoseEstimate();
        telemetry.addData("x",drive.pose.position.x);
        telemetry.addData("y",drive.pose.position.y);
        telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("test",drive.pose.toString());
        telemetry.update();

        Actions.runBlocking(
                new ParallelAction(
                        robot.goToHighGoal(),
                        trajOne
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        //robot.goToHighGoalSlides(),
                        robot.setIntakeRotate(FirstTeleOp.intakeRotateScore),
                        robot.outtake(),
                        robot.goToIntake(),
                        trajTwoHalf
                )
        );

       Actions.runBlocking(
               new SequentialAction(
                       robot.startIntakeSlow(),
                       robot.extendSlidesPower(-18000, -0.65, true),
                       robot.stopIntake(),
                       robot.retractSlidesPower(-8000,1, false),
                       robot.setIntakeRotate(0)
               )
       );

       Actions.runBlocking(
               new ParallelAction(
                       robot.goToHighGoal(),
                       trajThree
               )
       );

       Actions.runBlocking(
               new SequentialAction(
                       //robot.goToHighGoalSlides(),
                       robot.setIntakeRotate(FirstTeleOp.intakeRotateScore),
                       robot.outtake(),
                       robot.goToIntake(),
                       trajFour, // pickup second block
                       robot.startIntake(),
                       robot.extendSlidesPower(-19000, -0.65, true),
                       robot.stopIntake(),
                       robot.retractSlidesPower(-8000,1, false),
                       robot.setIntakeRotate(0)
               )
       );

        Actions.runBlocking(
                new ParallelAction(
                        robot.goToHighGoal(),
                        trajFive
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        //robot.goToHighGoalSlides(),
                        robot.setIntakeRotate(FirstTeleOp.intakeRotateScore),
                        robot.outtake(),
                        robot.goToIntake(),
                        trajSix, // pickup second block
                        robot.startIntake(),
                        robot.extendSlidesPower(-13000, -0.8, true),
                        robot.stopIntake(),
                        robot.retractSlidesPower(-8000,1, false),
                        robot.setIntakeRotate(0)
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        robot.goToHighGoal(),
                        trajSeven
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        //robot.goToHighGoalSlides(),
                        robot.setIntakeRotate(FirstTeleOp.intakeRotateScore),
                        robot.outtake(),
                        robot.goToIntake()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        trajEight,
                        robot.stopIntake()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        robot.extendSlidesPower(-8000,-1,false),
                        robot.startIntake(),
                        robot.extendSlidesPower(-22000,-0.65,true),
                        robot.setIntakeRotate(.2),
                        robot.retractSlidesPower(-12000, 1, false),
                        trajNine,
                        robot.extendSlidesPower(-22000,-0.65,true),
                        robot.setIntakeRotate(0.2)
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        robot.retractSlidesPower(-8000,1, false),
                        robot.stopIntake()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        robot.setIntakeRotate(0),
                        robot.GoToHighGoalAfter(400),
                        trajTen
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        //robot.goToHighGoalSlides(),
                        robot.setIntakeRotate(FirstTeleOp.intakeRotateScore),
                        robot.outtake()
                )
        );







/*
       Actions.runBlocking(
               new SequentialAction(
                       robot.startIntake(), // start intake
                       robot.extendSlidesPower(-13000, -0.4),
                       robot.extendSlidesPower(-16000, -0.35),
                       robot.stopIntake(),
                       trajThree, // deliver first block
                       robot.extendSlidesPower(-16000, -1),
                       robot.outtake(),
                       robot.retractSlidesPower(-13000, 1),
                       trajFour, // pickup second block
                       robot.startIntake(),
                       robot.extendSlidesPower(-15000, -0.4),
                       robot.stopIntake(),
                       trajFive, // deliver second block
                       robot.outtake(),
                       robot.retractSlidesPower(-13000, 1),
                       trajSix, // pickup third block
                       robot.startIntake(),
                       robot.extendSlidesPower(-15000, -0.4),
                       robot.stopIntake(),
                       robot.retractSlidesPower(-13000, 1),
                       trajSeven, // deliver third block
                       robot.extendSlidesPower(-15000, -1),
                       robot.outtake(),
                       robot.retractSlidesPower(-200, 1)
               )
       );

 */


        while(opModeIsActive())
        {
            drive.updatePoseEstimate();

            telemetry.addData("x",drive.pose.position.x);
            telemetry.addData("y",drive.pose.position.y);
            telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("test",drive.pose.toString());
            telemetry.addData("slides", backLeft.getCurrentPosition());

            telemetry.update();
        }



        if (isStopRequested()) return;
    }

    public class ScoreSystem
    {
        private DcMotorEx slideLeft1 = null;
        private DcMotorEx slideLeft2 = null;
        private DcMotorEx slideRight = null;
        private DcMotorEx pivot = null;

        private DcMotor backRight = null;
        private DcMotor backLeft = null;

        private CRServo intake = null;
        private Servo intakeRotate = null;

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

        public int slidesHigh = -26000;
        public int pivotScore = 800;
        public double intakeRotateScore = 0.35;

        public int pivotSpecimen = 500;
        public double rotatePosSpecimen = 0.3;
        public int slidesTargetSpecimen = -8000;
        public int slidesTargetSpecimenAfter = -15000;


        public ScoreSystem(HardwareMap hardwareMap)
        {
            backLeft = hardwareMap.get(DcMotor.class, "bl");
            backRight = hardwareMap.get(DcMotor.class, "br");

            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
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

            limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intake = hardwareMap.get(CRServo.class, "intake");
            intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");

        }

        public void setSlidePower(double power)
        {
            slideLeft1.setPower(power);
            slideLeft2.setPower(power);
            slideRight.setPower(power);

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

        public int getSlidesPosition()
        {
            return backLeft.getCurrentPosition();
        }

        public class Intake implements Action
        {
            private ElapsedTime timer = null;
            private boolean initialized = false;
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!initialized)
                {
                    timer = new ElapsedTime();
                    initialized = true;
                }
                intakeRotate.setPosition((1.184 * Math.pow(10, -10)) * (Math.pow(getSlidesPosition(), 2)) + (7.237 * Math.pow(10, -8)) * getSlidesPosition() + 0.055);
                intake.setPower(1);
                if(timer.milliseconds() < 800) {
                    return true;
                }
                else {
                    intake.setPower(0);
                    return false;
                }
            }
        }

        public Action intake()
        {
            return new Intake();
        }

        public class startIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(1);
                return false;
            }
        }

        public Action startIntake()
        {
            return new startIntake();
        }

        public class stopIntake implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(0);
                return false;
            }
        }

        public Action stopIntake()
        {
            return new stopIntake();
        }

        public class Outtake implements Action
        {
            private ElapsedTime timer = null;
            private boolean initialized = false;
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-1);
                if(!initialized)
                {
                    timer = new ElapsedTime();
                    initialized = true;
                }

                if(timer.milliseconds() < 500)
                    return true;
                else {
                    intake.setPower(0);
                    return false;
                }
            }
        }

        public Action outtake()
        {
            return new Outtake();
        }

        public class OuttakeAfter implements Action
        {
            private int ms;
            private ElapsedTime timer = null;
            private boolean initialized = false;
            public OuttakeAfter(int ms) {
                this.ms = ms;
            }
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!initialized)
                {
                    timer = new ElapsedTime();
                    initialized = true;
                }

                if(timer.milliseconds() < ms) {
                    return true;
                }

                else {
                    intake.setPower(-.3);
                    return false;
                }
            }
        }

        public Action outtakeAfter(int ms)
        {
            return new OuttakeAfter(ms);
        }


        public class extendSlidesPower implements Action {
            private int target;
            private double power;
            private boolean intake;
            public extendSlidesPower(int target, double power, boolean intake)
            {
                this.target = target;
                this.power = power;
                this.intake = intake;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(backLeft.getCurrentPosition() > target)
                {
                    if(intake)
                        intakeRotate.setPosition((1.184 * Math.pow(10, -10)) * (Math.pow(getSlidesPosition(), 2)) + (7.237 * Math.pow(10, -8)) * getSlidesPosition() + 0.055);
                    setSlidePower(power);
                    return true;
                }
                else
                {
                    setSlidePower(0);
                    return false;
                }
            }
        }

        public Action extendSlidesPower(int target, double power, boolean intake)
        {
            return new extendSlidesPower(target, power, intake);
        }


        public class retractSlidesPower implements Action {
            private int target;
            private double power;
            private boolean intake;
            public retractSlidesPower(int target, double power, boolean intake)
            {
                this.target = target;
                this.power = power;
                this.intake = intake;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(intake)
                    intakeRotate.setPosition((1.184 * Math.pow(10, -10)) * (Math.pow(getSlidesPosition(), 2)) + (7.237 * Math.pow(10, -8)) * getSlidesPosition() + 0.055);
                if(backLeft.getCurrentPosition() < target)
                {
                    setSlidePower(power);
                    return true;
                }
                else
                {
                    setSlidePower(0);
                    return false;
                }
            }
        }

        public Action retractSlidesPower(int target, double power , boolean intake)
        {
            return new retractSlidesPower(target, power, intake);
        }

        public class movePivotUp implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(backRight.getCurrentPosition() < 800)
                {
                    pivot.setPower(PivotPIDControl(800,backRight.getCurrentPosition()));
                    return true;
                }
                else
                {
                    pivot.setPower(0);
                    return false;
                }
            }
        }

        public Action movePivotUp()
        {
            return new movePivotUp();
        }

        public class movePivotDown implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(backRight.getCurrentPosition() > 100)
                {
                    pivot.setPower(PivotPIDControl(0,backRight.getCurrentPosition()));
                    return true;
                }
                else
                {
                    pivot.setPower(0);
                    return false;
                }
            }
        }

        public Action movePivotDown()
        {
            return new movePivotDown();
        }

        public class setIntakeRotate implements Action
        {
            private double pos;
            public setIntakeRotate(double pos)
            {
                this.pos = pos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeRotate.setPosition(pos);
                return false;
            }
        }

        public Action setIntakeRotate(double pos)
        {
            return new setIntakeRotate(pos);
        }

        public class startIntakeSlow implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(.3);
                return false;
            }
        }

        public Action startIntakeSlow()
        {
            return new startIntakeSlow();
        }

        public class startOuttake implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(-.1);
                return false;
            }
        }

        public Action startOuttake()
        {
            return new startOuttake();
        }

        public class SetPivotPower implements Action
        {
            private double p;
            public SetPivotPower(double p)
            {
               this.p = p;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPower(p);
                return false;
            }
        }

        public Action setPivotPower(double p)
        {
            return new SetPivotPower(p);
        }

        public class setSlidesPowerX implements Action
        {
            private double p;
            public setSlidesPowerX(double p)
            {
                this.p = p;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setSlidePower(p);
                return false;
            }
        }

        public Action setSlidesPowerX(double p)
        {
            return new setSlidesPowerX(p);
        }

        public class goToHighGoal implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPower(PivotPIDControl(FirstTeleOp.pivotScore, backRight.getCurrentPosition()));

                if(backRight.getCurrentPosition() > 800) {
                    //setSlidePower(SlideUpPIDControl(-27000, getSlidesPosition()));
                    setSlidePower(-1);
                    if(getSlidesPosition() < -27000) {
                        setSlidePower(-.2);
                        pivot.setPower(.2);
                        return false;
                    }
                    else if(getSlidesPosition() < -15000 && getSlidesPosition() > -27000) {
                        intakeRotate.setPosition(FirstTeleOp.intakeRotateScore);
                        return true;
                    }
                    else
                        return true;
                }
                else
                    return true;

            }
        }

        public class goToHighGoalSlides implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //setSlidePower(SlideUpPIDControl(FirstTeleOp.slidesHigh, getSlidesPosition()));
                setSlidePower(-1);
                if(getSlidesPosition() < -27000) {
                    setSlidePower(-.2);
                    return false;
                }
                else
                    return true;
            }
        }

        public class goToIntake implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if(backRight.getCurrentPosition() < 50 && getSlidesPosition() > -200)
                {
                    pivot.setPower(0);
                    setSlidePower(0);
                    return false;
                }
                else {
                    pivot.setPower(PivotPIDControl(20, backRight.getCurrentPosition()));
                    setSlidePower(SlideUpPIDControl(-200, getSlidesPosition()));
                    return false;
                }

            }
        }

        public Action goToHighGoal()
        {
            return new goToHighGoal();
        }

        public Action goToIntake() {
            return new goToIntake();
        }

        public Action goToHighGoalSlides()
        {
            return new goToHighGoalSlides();
        }

        public class setAllZero implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivot.setPower(0);
                setSlidePower(0);
                return false;
            }
        }

        public Action setAllZero()
        {
            return new setAllZero();
        }

        public class extendSlidesPowerFirst implements Action {
            private int target;
            private double power;
            public extendSlidesPowerFirst(int target, double power)
            {
                this.target = target;
                this.power = power;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(backLeft.getCurrentPosition() > target)
                {
                    intakeRotate.setPosition(0.07);
                    setSlidePower(power);
                    return true;
                }
                else
                {
                    setSlidePower(0);
                    return false;
                }
            }
        }

        public Action extendSlidesPowerFirst(int target, double power)
        {
            return new extendSlidesPowerFirst(target, power);
        }


        public class GoToHighGoalAfter implements Action
        {
            private int ms;
            private ElapsedTime timer = null;
            private boolean initialized = false;
            public GoToHighGoalAfter(int ms) {
                this.ms = ms;
            }
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!initialized)
                {
                    timer = new ElapsedTime();
                    initialized = true;
                }

                if(timer.milliseconds() < ms) {
                    return true;
                }

                else {
                    pivot.setPower(PivotPIDControl(FirstTeleOp.pivotScore, backRight.getCurrentPosition()));

                    if(backRight.getCurrentPosition() > 600) {
                        //setSlidePower(SlideUpPIDControl(-27000, getSlidesPosition()));
                        setSlidePower(-1);
                        if(getSlidesPosition() < -27000) {
                            setSlidePower(-.2);
                            pivot.setPower(.2);
                            return false;
                        }
//                        else if(getSlidesPosition() < -15000 && getSlidesPosition() > -26000) {
//                            intakeRotate.setPosition(FirstTeleOp.intakeRotateScore);
//                            return true;
//                        }
                        else {
                            return true;
                        }
                    }
                    else
                        return true;
                }
            }
        }

        public Action GoToHighGoalAfter(int ms)
        {
            return new GoToHighGoalAfter(ms);
        }
    }
}