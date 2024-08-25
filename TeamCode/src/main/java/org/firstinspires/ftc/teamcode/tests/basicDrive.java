package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class basicDrive extends LinearOpMode {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    SampleMecanumDrive drive;

    private DcMotor intakeSpinner;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        fl = hardwareMap.get(DcMotor.class, "frontleft");
        fr = hardwareMap.get(DcMotor.class, "frontright");
        bl = hardwareMap.get(DcMotor.class, "backleft");
        br = hardwareMap.get(DcMotor.class, "backright");
        boolean noIntakeSpinner = false;
        try {
            intakeSpinner = hardwareMap.get(DcMotor.class, "intakeSpinner");
        }
        catch (Exception e) {
            noIntakeSpinner = true;
        }
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double x;
        double y;
        double pivot;
        double intakePower;

        waitForStart();
        while (opModeIsActive()) {
            drive.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));

            intakePower = (gamepad1.right_trigger/2) - (gamepad1.left_trigger/2);
            if (!noIntakeSpinner) {
                intakeSpinner.setPower(intakePower);
            }

            telemetry.addData("Joystick y", gamepad1.left_stick_y);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("parallel", fl.getCurrentPosition());
            packet.put("perpendicular", fr.getCurrentPosition());
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
