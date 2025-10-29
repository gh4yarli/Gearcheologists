package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous
public class RedLoadingZone extends LinearOpMode {

    @Override
    public void runOpMode(){

        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");
        CRServo left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
        CRServo right_feeder = hardwareMap.get(CRServo.class, "right_feeder");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        Pose2d startingPose = new Pose2d(-60,-12, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap,startingPose);
        pinpoint.initialize();
        Pose2D pos;
        waitForStart();
        intake.setPower(0);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -60, -12, AngleUnit.DEGREES, 0));
        Action path = drive.actionBuilder(startingPose)
                .splineTo(new Vector2d(-5,-11), Math.toRadians(-55))
                .build();
        Actions.runBlocking(new SequentialAction(path));
        launcher.setPower(-0.55);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (timer.milliseconds() < 3000){
            sleep(1);
        }
        intake.setPower(0.6);
        timer.reset();
        for (byte i = 0; i < 3; i++) {
            left_feeder.setPower(1);
            right_feeder.setPower(-1);
            while (timer.milliseconds() < 525){
                sleep(1);
            }
            timer.reset();
            left_feeder.setPower(0);
            right_feeder.setPower(0);
            while (timer.milliseconds() < 1500){
                sleep(1);
            }
            timer.reset();
            }
        launcher.setPower(0);
        intake.setPower(0);
        while (opModeIsActive()){
            pinpoint.update();
            pos = pinpoint.getPosition();
            telemetry.addData("Position", String.format(Locale.US,
                    "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.INCH),
                    pos.getY(DistanceUnit.INCH),
                    pos.getHeading(AngleUnit.DEGREES)));
            telemetry.update();
        }
    }
}

