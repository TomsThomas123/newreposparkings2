package org.firstinspires.ftc.teamcode.hardware;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Outtake {


    private DcMotor Outtake;
    private OpMode myOpMode;


    public Outtake(OpMode opMode) {
        myOpMode = opMode;


    }
    public void init() {
        Outtake = hardwareMap.get(DcMotor.class, "Outake");
    }


    public class Outtakeforeward implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Outtake.setPower(0.8);
            return false;
        }
    }
    public Action Outakeforeward() { return new Outtakeforeward(); }
    public class Stop implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Outtake.setPower(0);
            return false;
        }
    }
    public Action Stop() {return new Stop();}
}

