package org.firstinspires.ftc.teamcode.meet2;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class M2_JustMove extends LinearOpMode {

    @Override
    public void runOpMode(){
        Pose2d startingPose = new Pose2d(0,0,0) ;
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(15)
                .build();
        waitForStart();
        Actions.runBlocking(new SequentialAction(path));
        stop();
    }
}
