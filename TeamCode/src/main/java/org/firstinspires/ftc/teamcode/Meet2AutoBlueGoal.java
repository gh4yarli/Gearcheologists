package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class Meet2AutoBlueGoal extends LinearOpMode {
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
        leftLauncher.setPower(-0.3533);
        rightLauncher.setPower(0.3533);
        intakeMotor.setPower(1);
        feederMotor.setPower(1);
        Pose2d startingPose = new Pose2d(new Vector2d(52, 44), Math.toRadians(45));
        Pose2d preloadedArtifacts = new Pose2d(new Vector2d(14, 48), Math.toRadians(-135));
        drive = new MecanumDrive(hardwareMap, startingPose);
        Action moveShoot = drive.actionBuilder(new Pose2d(52, 44, Math.toRadians(45)))
                .lineToX(9)
                .build();
        Actions.runBlocking(moveShoot);
        for (int i = 0; i < 4; i++) {
            rightFeeder.setPower(-1);
            leftFeeder.setPower(-1);
            sleep(750);
            rightFeeder.setPower(0);
            leftFeeder.setPower(0);
        }
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);
        Action collect = drive.actionBuilder(new Pose2d(9, 7.615386, Math.toRadians(-135)))
                .turnTo(Math.toRadians(0))
                .lineToX(-5)
                .turnTo(Math.toRadians(-90))
                .lineToY(65)
                .lineToY(20)
                .turnTo(Math.toRadians(30))
                .build();
        Actions.runBlocking(collect);
    }
}