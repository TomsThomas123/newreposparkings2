package org.firstinspires.ftc.teamcode.hardware;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class InTake {


    private DcMotor intake;
    private OpMode myOpMode;


    public InTake(OpMode opMode) {
        myOpMode = opMode;


    }
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }
    public class IntakeForward implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(1);
            return false;
        }
    }
    public Action IntakeForward() {
        return new IntakeForward();
    }


    public class IntakeBackwards implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(-1);
            return false;
        }
    }
    public Action IntakeBackwards() {
        return new IntakeBackwards();
    }
    public class Stop implements Action {
        private boolean initialized = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(0);
            return false;
        }
    }
    public Action Stop() {
        return new Stop();
    }
}

