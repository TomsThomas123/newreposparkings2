package org.firstinspires.ftc.teamcode.auto2526;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="encoderbackup", group="Robot")


public class encoderbackup extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake = null;
    private DcMotor outake1 = null;
    private DcMotor outake2 = null;
    private DcMotor transfer1 = null;

    private Servo   stop      = null;

    private ElapsedTime     runOpMode = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.09449 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    final double INTAKEGO = 1.0;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException{

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_Left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_Left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_Right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_Right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outake1 = hardwareMap.get(DcMotor.class, "outake1");
        outake2 = hardwareMap.get(DcMotor.class, "outake2");
        transfer1 = hardwareMap.get(DcMotor.class, "transfer99");
        stop = hardwareMap.get(Servo.class, "stop");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeftDrive.getCurrentPosition(),
                backLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition(),
                backRightDrive.getCurrentPosition());
        telemetry.update();

        imu.initialize(parameters);

        waitForStart
                ();
        outake1.setPower(1);

        ForwardBackward(0.5,11, -1);
        intake.setPower(1);
        transfer1.setPower(-1);
        intake.setPower(1);







        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(1000);  // pause to display final telemetry message.
        requestOpModeStop();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */


    public void ForwardBackward(double speed, double inches, double movement) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        intake.setPower(1);

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = frontLeftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * movement);
            newRightFrontTarget = frontRightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * movement);
            newLeftBackTarget = backLeftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * movement);
            newRightBackTarget = backRightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH * movement);
            frontLeftDrive.setTargetPosition(newLeftFrontTarget);
            frontRightDrive.setTargetPosition(newRightFrontTarget);
            backLeftDrive.setTargetPosition(newLeftBackTarget);
            backRightDrive.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (frontLeftDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy() || frontRightDrive.isBusy())) {

            }


            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            intake.setPower(1);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // optional pause after each move.
        }
    }


    public void Left(double speed, double inches) {


        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        intake.setPower(1);

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = frontLeftDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * -1);
            newRightFrontTarget = frontRightDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH );
            newLeftBackTarget = backLeftDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = backRightDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * -1);
            frontLeftDrive.setTargetPosition(newLeftFrontTarget);
            frontRightDrive.setTargetPosition(newRightFrontTarget);
            backLeftDrive.setTargetPosition(newLeftBackTarget);
            backRightDrive.setTargetPosition(newRightBackTarget);
            intake.setPower(1);


            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (frontLeftDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy() || frontRightDrive.isBusy())) {

            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            intake.setPower(1);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void Right(double speed, double inches) {


        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = frontLeftDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH  * 1);
            newRightFrontTarget = frontRightDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * -1 );
            newLeftBackTarget = backLeftDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * -1 );
            newRightBackTarget = backRightDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * 1);
            frontLeftDrive.setTargetPosition(newLeftFrontTarget);
            frontRightDrive.setTargetPosition(newRightFrontTarget);
            backLeftDrive.setTargetPosition(newLeftBackTarget);
            backRightDrive.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (frontLeftDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy() || frontRightDrive.isBusy())) {

            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            intake.setPower(1);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    public void liftMotorPlacement(double speed, int liftposition) {


        // Ensure that the OpMode is still active



    }
    /* public void turnGyro( double speed, int howmanydegrees){
         YawPitchRollAngles e = imu.getRobotYawPitchRollAngles();
         int robotDegrees = (int) e.getYaw(AngleUnit.DEGREES);
         int degreeTarget =  howmanydegrees;
         while(opModeIsActive()) {
             while (degreeTarget != robotDegrees) {
                 frontLeftDrive.setPower(speed);
                 frontRightDrive.setPower(-speed);
                 backLeftDrive.setPower(speed);
                 backRightDrive.setPower(-speed);
             }
         }
         if (degreeTarget == robotDegrees){
             frontLeftDrive.setPower(0);
             frontRightDrive.setPower(0);
             backLeftDrive.setPower(0);
             backRightDrive.setPower(0);
         }
     }*/
    public void turnLeft(double power, long time) {
        frontLeftDrive.setPower(-power);
        backLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
        intake.setPower(1);
        sleep(time);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        intake.setPower(1);
    }
    public void turnRight(double power, long time) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backRightDrive.setPower(-power);
        intake.setPower(1);
        sleep(time);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        intake.setPower(1);
    }


}
