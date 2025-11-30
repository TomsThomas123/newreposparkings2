package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.hardware.Outtake;

@TeleOp
public class pidflearn extends OpMode {
    private PIDController controller;
    private MotorFeedforward

    public static double p=0, i=0, d=0;
    public static double f=0;

    public static int target = 0;

    private Outtake;

    private DcMotorEx Outtake;

    @Override
    public void init() {
        controller = new PIDController(p, i ,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor0");

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toIntExact(target / ))
    }
}
