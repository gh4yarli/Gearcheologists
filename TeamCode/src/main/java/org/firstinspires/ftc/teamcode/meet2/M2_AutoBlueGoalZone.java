package org.firstinspires.ftc.teamcode.meet2;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
@Disabled
public class M2_AutoBlueGoalZone extends LinearOpMode {
    private DcMotor leftLauncher, rightLauncher;
    private CRServo leftFeeder, rightFeeder;
    private DcMotor feederMotor, intakeMotor;
    private MecanumDrive drive;
    @Override
    public void runOpMode() {
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        feederMotor = hardwareMap.get(DcMotor.class, "feeder");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake2");
        leftLauncher.setPower(-0.346);
        rightLauncher.setPower(0.346);
        intakeMotor.setPower(1);
        feederMotor.setPower(1);
        Pose2d startingPose = new Pose2d(new Vector2d(52, 44), Math.toRadians(45));
        Pose2d preloadedArtifacts = new Pose2d(new Vector2d(14, 48), Math.toRadians(-135));
        drive = new MecanumDrive(hardwareMap, startingPose);
        Action moveShoot = drive.actionBuilder(new Pose2d(52, 44, Math.toRadians(45)))
                .lineToX(20)
                .build();
        Actions.runBlocking(moveShoot);
        for (int i = 0; i < 6; i++) {
            rightFeeder.setPower(-1);
            leftFeeder.setPower(1);
            sleep(750);
            rightFeeder.setPower(0);
            leftFeeder.setPower(0);
        }
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);
        drive.localizer.update();
        Pose2d collectMove = drive.localizer.getPose();
        Action collect = drive.actionBuilder(collectMove)
                .turnTo(Math.toRadians(0))
                .lineToX(10)
                .turnTo(Math.toRadians(270))
                .lineToY(70)
                .lineToY(20)
                .turnTo(Math.toRadians(45))
                .build();
        Actions.runBlocking(collect);
    }
}