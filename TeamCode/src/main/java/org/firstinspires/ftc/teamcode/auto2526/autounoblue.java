package org.firstinspires.ftc.teamcode.auto2526;


// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.InTake;


@Config
@Autonomous(name = "SpecimenAUTONEW", group = "Autonomous")
public class autounoblue extends LinearOpMode {
    Pose2d startPose;
    MecanumDrive drive;

    Outtake outtake = new Outtake(this);
    Transfer transfer = new Transfer(this);
    InTake inTake= new InTake(this);



    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose2d(61, -57, Math.toRadians(115));
        drive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder build = drive.actionBuilder(startPose)

                .splineToSplineHeading(new Pose2d(new Vector2d(48,-48), Math.toRadians(125)), 0)
                .afterTime(0.1, outtake.Outakeforeward())
                .afterTime(3, transfer.transferforeward())
                .afterTime(1, inTake.IntakeForward())
                .afterTime(2, transfer.Stop())

                .splineToSplineHeading(new Pose2d(new Vector2d(14,-18), Math.toRadians(180)), 0)
                .splineToSplineHeading(new Pose2d(new Vector2d(14,-66), Math.toRadians(180)), 0)

                .afterTime(0.1, outtake.Outakeforeward())
                .afterTime(3, transfer.transferforeward())
                .afterTime(1, inTake.IntakeForward())
                .afterTime(2, transfer.Stop())
                .splineToSplineHeading(new Pose2d(new Vector2d(48,-48), Math.toRadians(115)), 0)
                ;

        int position= 1;
        while (!isStopRequested() && !opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(

            ));


        }

        waitForStart();
        Actions.runBlocking(new SequentialAction(
                build.build()
        ));









    }

}

