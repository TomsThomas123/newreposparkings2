package org.firstinspires.ftc.teamcode.hardware;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Transfer {


    private DcMotor transfer;
    private OpMode myOpMode;


    public Transfer(OpMode opMode) {
        myOpMode = opMode;


    }
    public void init() {
        transfer = hardwareMap.get(DcMotor.class, "transfer");
    }
    public class transferforeward implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transfer.setPower(1);
            return false;
        }
    }
    public Action transferforeward() {
        return new transferforeward();
    }


    public class transferSlow implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transfer.setPower(.2);
            return false;
        }
    }
    public Action transferSlow() {
        return new transferSlow();
    }
    public class Stop implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transfer.setPower(0);
            return false;
        }
    }
    public Action Stop() {
        return new Stop();
    }
}

