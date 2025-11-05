package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp
public class FieldCentricTest extends OpMode {

    GoBildaPinpointDriver pinpoint;
    DcMotorEx bl, fl, fr, br;
    @Override
    public void init() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Names.ODOMETRY_COMPUTER);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setOffsets(83.0, -115.0, DistanceUnit.MM);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        bl = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        fl = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        fr = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        br = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop(){
        pinpoint.update();
        if (gamepad1.a) {
            pinpoint.resetPosAndIMU();
        }

        Pose2D pos = pinpoint.getPosition();
        double heading = -pinpoint.getHeading(AngleUnit.RADIANS);

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        //Switched X and Y
        double rotX = strafe * Math.cos(heading) - forward * Math.sin(heading);
        double rotY = strafe * Math.sin(heading) + forward * Math.cos(heading);

        double pfl = rotY + rotX + rotate;
        double pfr = rotY - rotX - rotate;
        double pbl = rotY - rotX + rotate;
        double pbr = rotY + rotX - rotate;

        fl.setPower(pfl / 2);
        fr.setPower(pfr / 2);
        bl.setPower(pbl / 2);
        br.setPower(pbr / 2);
        telemetry.addData("Position", "x: %.1f  y: %.1f  h: %.1f°",
                pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Encoders", "X: %d  Y: %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
        telemetry.update();
    }
}