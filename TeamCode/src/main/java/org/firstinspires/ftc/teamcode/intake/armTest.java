package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class armTest extends OpMode {
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    arm arm;
    double armPower = 0;
    @Override
    public void init() {
        arm = new arm(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        arm.updatePosition();
        armPower = (gamepad1.right_trigger - gamepad1.left_trigger)*0.5;
        arm.setPower(armPower);

        dashboardTelemetry.addData("Arm Position", arm.getPos());
        dashboardTelemetry.update();
        telemetry.addData("Arm Position", arm.getPos());
        telemetry.update();
    }
}
