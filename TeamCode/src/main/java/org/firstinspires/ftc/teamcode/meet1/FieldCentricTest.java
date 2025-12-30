package org.firstinspires.ftc.teamcode.meet1;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
@Disabled

public class FieldCentricTest extends OpMode {

    GoBildaPinpointDriver odo;
    DcMotorEx bl, fl, fr, br;
    @Override
    public void init() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.setOffsets(83.0, -115.0, DistanceUnit.MM);
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        bl = hardwareMap.get(DcMotorEx.class, "left_back");
        fl = hardwareMap.get(DcMotorEx.class, "left_front");
        fr = hardwareMap.get(DcMotorEx.class, "right_front");
        br = hardwareMap.get(DcMotorEx.class, "right_back");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop(){
        odo.update();
        if (gamepad1.a) {
            double x = odo.getPosX(DistanceUnit.MM);
            double y = odo.getPosY(DistanceUnit.MM);
            odo.resetPosAndIMU();
            odo.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, 0));
        }

        Pose2D pos = odo.getPosition();
        double heading = -odo.getHeading(AngleUnit.RADIANS);

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        //Switched X and Y
        double rotX = strafe * Math.cos(heading) - forward * Math.sin(heading);
        double rotY = strafe * Math.sin(heading) + forward * Math.cos(heading);

        /*double pfl = -rotX + rotY + rotate;
        double pfr = -rotX - rotY - rotate;
        double pbl = -rotX - rotY + rotate;
        double pbr = -rotX + rotY - rotate;*/
        double pfl = rotY - rotX + rotate;
        double pfr = rotY + rotX - rotate;
        double pbl = rotY + rotX + rotate;
        double pbr = rotY - rotX - rotate;

        fl.setPower(pfl / 2);
        fr.setPower(pfr / 2);
        bl.setPower(pbl / 2);
        br.setPower(pbr / 2);
        telemetry.addData("Position", "x: %.1f  y: %.1f  h: %.1f°",
                pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Encoders", "X: %d  Y: %d", odo.getEncoderX(), odo.getEncoderY());
        telemetry.update();
    }
}