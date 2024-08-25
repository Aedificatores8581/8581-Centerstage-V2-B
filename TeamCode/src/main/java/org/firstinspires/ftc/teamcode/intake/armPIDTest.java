package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ButtonDown;

@TeleOp
public class armPIDTest extends LinearOpMode {
    arm arm;
    Claw claw;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    int targetPosition = 0;
    @Override
    public void runOpMode() {
        arm = new arm(hardwareMap);
        claw = new Claw(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        ButtonDown buttonDown = new ButtonDown(() -> {targetPosition += 100;});
        ButtonDown buttonUp = new ButtonDown(() -> {targetPosition -= 100;});

        while (!opModeIsActive()) {
            double armPower = (gamepad1.right_trigger - gamepad1.left_trigger)*0.5;
            arm.setPower(armPower);
            if (isStopRequested()) {break;}
        }
        waitForStart();

        ElapsedTime runTime = new ElapsedTime();
        double buttonTime =0;
        arm.resetEncoder();

        while (opModeIsActive()) {
            double armPower = (gamepad1.right_trigger - gamepad1.left_trigger)*0.5;
            arm.setPower(armPower);
            arm.updatePosition();

            /*if (gamepad1.dpad_up && runTime.seconds() > buttonTime+0.25) {
                targetPosition += 100;buttonTime = runTime.seconds();
            }
            else if (gamepad1.dpad_down && runTime.seconds() > buttonTime+0.25) {
                targetPosition -= 100;buttonTime = runTime.seconds();
            }*/
            buttonDown.update(gamepad1.dpad_down);
            buttonUp.update(gamepad1.dpad_up);
            if (targetPosition > 0) targetPosition = 0;

            if (gamepad1.a) {
                //arm.runToPosition(targetPosition);
                runToPosition(targetPosition);

            }

            double degreesOff = (double) arm.getError(); //Get Initial Error
            degreesOff *= 360.0 / 8192.0; //Convert to Degree
            double wristPosition = degreesOff / 45.0 + 0.15;
            claw.wrist.setPositionRaw(wristPosition);

            dashboardTelemetry.addData("Pos to Go", targetPosition);
            dashboardTelemetry.addData("Target Position", arm.getTargetPos());
            dashboardTelemetry.addData("Arm Position", arm.getPos());
            dashboardTelemetry.addData("Arm Encoder Position", arm.encoder.getPosition());
            dashboardTelemetry.addData("Error", arm.getError());
            dashboardTelemetry.addData("down power", arm.downPwr());
            dashboardTelemetry.addData("up power", arm.raiseMath());
            dashboardTelemetry.addData("Wrist Pos", wristPosition);
            dashboardTelemetry.addData("Degrees Off", degreesOff);
            //dashboardTelemetry.addData("runToMethod", arm.runToMethod);
            dashboardTelemetry.update();
            telemetry.addData("Pos to Go", targetPosition);
            telemetry.addData("Target Position", arm.getTargetPos());
            telemetry.addData("Arm Position", arm.getPos());
            telemetry.addData("Arm Encoder Position", arm.encoder.getPosition());
            telemetry.addData("Error", arm.getError());
            telemetry.addData("down power", arm.downPwr());
            telemetry.addData("up power", arm.raiseMath());
            telemetry.addData("Wrist Pos", wristPosition);
            telemetry.addData("Degrees Off", degreesOff);
            //telemetry.addData("runToMethod", arm.runToMethod);
            telemetry.update();
        }
    }
    public Object stopRequested() {
        return isStopRequested();
    }
    public Object telemetryPass() {
        double error = (arm.getTargetPos() - arm.getPos());
        telemetry.addData("Error", error);
        telemetry.addData("run to method", arm.runToMethod);
        telemetry.update();
        dashboardTelemetry.addData("Error", error);
        dashboardTelemetry.addData("run to method", arm.runToMethod);
        dashboardTelemetry.update();
        return 1;
    }
    public void runToPosition(int position) {
        arm.runToPositionAsync(position);
        while (arm.isBusy()) {
            telemetryPass();
            arm.update();
            if (isStopRequested()) {return;}
        }
        arm.updatePosition();if (arm.isBusy()) {runToPosition(position);}
        arm.setPower(0);
    }
}
