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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class AutonomousTest extends LinearOpMode {

    public double zeroPoint;
    public AnalogInput canandgyro;
    public double botHeading;
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        canandgyro = hardwareMap.get(AnalogInput.class,"canandgyro");
        zeroPoint = canandgyro.getVoltage();

        Pose2d currentPose = null;

        TrajectoryActionBuilder finalTraj = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-26.26, 0)) // deliver preload specimen
                .waitSeconds(3)
                .strafeTo(new Vector2d(-16.26, 0)) // back up
                .strafeTo(new Vector2d(-16.26,-12)) // move left
                .strafeToLinearHeading(new Vector2d(-24.6, -12), Math.toRadians(-122)) // move forward to first sample pickup
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-16.63, -25.61),Math.toRadians(-50)) // deliver first sample
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-24.6,-22), Math.toRadians(-122)) // pickup second
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-16.63, -25.61),Math.toRadians(-50)) // deliver second
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-24.6, -32.7), Math.toRadians(-122)) // pickup third
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-16.63, -25.61),Math.toRadians(-50)) // deliver third
                .waitSeconds(3)
                ;




        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        Action trajectoryActionChosen = finalTraj.build();

        telemetry.update();
        waitForStart();


        drive.updatePoseEstimate();
        telemetry.addData("x",drive.pose.position.x);
        telemetry.addData("y",drive.pose.position.y);
        telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("test",drive.pose.toString());
        telemetry.update();

        Actions.runBlocking(trajectoryActionChosen);

        while(opModeIsActive())
        {
            botHeading = 2* AngleUnit.normalizeDegrees(canandgyro.getVoltage() - zeroPoint);;
            drive.updatePoseEstimate();

            telemetry.addData("x",drive.pose.position.x);
            telemetry.addData("y",drive.pose.position.y);
            telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("test",drive.pose.toString());

            telemetry.addData("canandgyro",botHeading);

            telemetry.update();
        }



        if (isStopRequested()) return;

        // Actions.runBlocking(trajectoryActionChosen);

    }
}