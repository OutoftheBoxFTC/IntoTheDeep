package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "PositionTest", group = "Autonomous")
public class PositionTest extends LinearOpMode {

    public DcMotorEx backLeft;
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ScoreSystem robot = new ScoreSystem(hardwareMap);

        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        Pose2d currentPose = null;

        TrajectoryActionBuilder finalTraj = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-24.8, 0)) // deliver preload specimen
                .strafeTo(new Vector2d(-16.26, 0)) // back up
                .strafeToLinearHeading(new Vector2d(-25.3, 20.2), Math.toRadians(121.5)) // move forward to first sample pickup
                .strafeToLinearHeading(new Vector2d(-16.9, 24.2),Math.toRadians(45.6)) // deliver first sample
                .strafeToLinearHeading(new Vector2d(-26.8,29.1), Math.toRadians(117.3)) // pickup second
                .strafeToLinearHeading(new Vector2d(-19.2, 31.1),Math.toRadians(50)) // deliver second
                .strafeToLinearHeading(new Vector2d(-28.3, 38), Math.toRadians(116.6)) // pickup third
                .strafeToLinearHeading(new Vector2d(-19.2, 38),Math.toRadians(50)) // deliver third

                .strafeToLinearHeading(new Vector2d(-8.3, 28), Math.toRadians(0))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-27, -1.9), Math.toRadians(0))
                .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(-8.3, 28), Math.toRadians(0))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-27, -1.9), Math.toRadians(0))
                .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(-8.3, 28), Math.toRadians(0))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-27, -1.9), Math.toRadians(0))
                .waitSeconds(3)
                ;


        TrajectoryActionBuilder one = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-24.8, 0)) // deliver preload specimen
                ;

        TrajectoryActionBuilder two = one.endTrajectory().fresh()
                .strafeTo(new Vector2d(-16.26, 0)) // back up
                .strafeToLinearHeading(new Vector2d(-25.3, 20.2), Math.toRadians(121.5)) // move forward to first sample pickup
                ;

        TrajectoryActionBuilder three = two.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-16.9, 24.2),Math.toRadians(45.6)) // deliver first sample
                ;

        TrajectoryActionBuilder four = three.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-26.8,29.1), Math.toRadians(117.3)) // pickup second
                ;

        TrajectoryActionBuilder five = four.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-19.2, 31.1),Math.toRadians(50)) // deliver second
                ;

        TrajectoryActionBuilder six = five.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-28.3, 38), Math.toRadians(116.6)) // pickup third
                ;

        TrajectoryActionBuilder seven = six.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-19.2, 38),Math.toRadians(50)) // deliver third
                ;

        TrajectoryActionBuilder eight = seven.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8.3, 28), Math.toRadians(0))
                ;

        TrajectoryActionBuilder nine = eight.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-27, -1.9), Math.toRadians(0))
                ;
        TrajectoryActionBuilder ten = nine.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8.3, 28), Math.toRadians(0))
                ;

        TrajectoryActionBuilder eleven = ten.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-27, -1.9), Math.toRadians(0))
                ;
        TrajectoryActionBuilder twelve = eleven.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8.3, 28), Math.toRadians(0))
                ;

        TrajectoryActionBuilder thirteen = twelve.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-27, -1.9), Math.toRadians(0))
                ;

        TrajectoryActionBuilder fourteen = thirteen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-16.9, 24.2),Math.toRadians(45.6))
                ;


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        Action trajectoryActionChosen = finalTraj.build();
        Action trajOne = one.build();
        Action trajTwo = two.build();
        Action trajThree = three.build();
        Action trajFour = four.build();
        Action trajFive = five.build();
        Action trajSix = six.build();
        Action trajSeven = seven.build();
        Action trajEight = eight.build();
        Action trajNine = nine.build();
        Action trajTen = ten.build();
        Action trajEleven = eleven.build();
        Action trajTwelve = twelve.build();
        Action trajThirteen = thirteen.build();
        Action trajFourteen = fourteen.build();


        telemetry.update();
        waitForStart();


        drive.updatePoseEstimate();
        telemetry.addData("x",drive.pose.position.x);
        telemetry.addData("y",drive.pose.position.y);
        telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("test",drive.pose.toString());
        telemetry.update();

//        Actions.runBlocking(
//                new SequentialAction(
//                        trajOne,
//                        trajTwo,
//                        robot.extendSlides(),
//                        robot.intake(),
//                        trajThree,
//                        robot.outtake(),
//                        trajFour,
//                        robot.intake(),
//                        trajFive,
//                        robot.outtake(),
//                        trajSix,
//                        robot.intake(),
//                        robot.retractSlides(),
//                        trajSeven,
//                        robot.extendSlides(),
//                        robot.outtake(),
//                        robot.retractSlides(),
//                        trajEight,
//                        trajNine,
//                        trajTen,
//                        trajEleven,
//                        trajTwelve,
//                        trajThirteen,
//                        trajFourteen
//                )
//        );

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


        public class ExtendSlides implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setSlidePower(SlideDownPIDControl(-14784, backLeft.getCurrentPosition()));
                if(backLeft.getCurrentPosition() > -14000)
                    return true;
                else {
                    setSlidePower(0);
                    return false;
                }
            }

        }

        public Action extendSlides()
        {
            return new ExtendSlides();
        }

        public class RetractSlides implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setSlidePower(SlideDownPIDControl(0, backLeft.getCurrentPosition()));
                if(backLeft.getCurrentPosition() < -1000)
                    return true;
                else {
                    setSlidePower(0);
                    return false;
                }
            }
        }
        public Action retractSlides()
        {
            return new RetractSlides();
        }

        public class Intake implements Action
        {
            private ElapsedTime timer = new ElapsedTime();
            public boolean run(@NonNull TelemetryPacket packet) {
                if(timer.milliseconds() < 800) {
                    intakeRotate.setPosition((1.184 * Math.pow(10, -10)) * (Math.pow(getSlidesPosition(), 2)) + (7.237 * Math.pow(10, -8)) * getSlidesPosition() + 0.055);
                    intake.setPower(1);
                    return true;
                }
                else {
                    intakeRotate.setPosition(.25);
                    intake.setPower(0);
                    return false;
                }
            }
        }

        public Action intake()
        {
            return new Intake();
        }

        public class Outtake implements Action
        {
            private ElapsedTime timer = new ElapsedTime();
            public boolean run(@NonNull TelemetryPacket packet) {
                if(timer.milliseconds() < 800) {
                    intakeRotate.setPosition(.25);
                    intake.setPower(-1);
                    return true;
                }
                else {
                    intakeRotate.setPosition(.25);
                    intake.setPower(0);
                    return false;
                }
            }
        }

        public Action outtake()
        {
            return new Outtake();
        }



    }
}
