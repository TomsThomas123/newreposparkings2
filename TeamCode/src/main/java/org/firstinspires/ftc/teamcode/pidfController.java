package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.MotorFeedforward;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class pidfController extends OpMode {

    DcMotorEx motor;
    private PIDFController pidf;


    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kv = 0;



    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void init(){
        pidf = new PIDFController(Kp,Ki,Kd, Kv);
        pidf.reset();
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {

        double state = motor.getVelocity();
        telemetry.addData("Current Velocity:  Target:", "%.7f, %.7f", state, 1000);
        telemetry.update();


        double output = pidf.calculate(state, 1000);
        motor.setVelocity(output);



    }


}