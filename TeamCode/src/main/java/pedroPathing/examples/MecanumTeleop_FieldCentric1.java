package pedroPathing.examples;


import
com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp()
@Config
public class MecanumTeleop_FieldCentric1 extends LinearOpMode {


    OldHardwareMecanum robot = new OldHardwareMecanum();   // Use a Mecanum's hardware
    double panPosition = HardwareMecanum.panARM_HOME;  // servo's position
    final double droneARM_SPEED = 0.10;  // set rate to move servo
    double twistPosition = HardwareMecanum.twistClawARM_HOME;  // servo's position
    final double twistARM_SPEED = 0.005;  // set rate to move servo

    double clawPosition = HardwareMecanum.clawARM_HOME;  // servo's position
    final double clawARM_SPEED = 0.005;  // set rate to move servo

    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public BNO055IMU imu;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // int newTarget=robot.lift.getTargetPosition() + (int) COUNTS_PER_MOTOR_REV*2;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Defines the parameters for the gyro (units)
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Intializes the parameters previously defined
        imu.initialize(imuParameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.twistClaw.setPosition(0.30);
        robot.tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gets the angle from the imu**/
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            double angle = angles.firstAngle;

            //gets squared values from the driver's stick input**/
            double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            //finds the desired angle that the driver wants to move the robot**/
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            //sets the movement angle by finding the difference of the robots angle, the input angle and the offset value
            // the offset value is set by the the driver if the imu does not reset after auto*/
            robotAngle = robotAngle - Math.toRadians(angle);

            double rightX = gamepad1.right_stick_x;

            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            // Output the safe vales to the motor drives.++
            robot.frontLeftDrive.setPower(v4);    //motor3
            robot.frontRightDrive.setPower(v2);   //motor2
            robot.backLeftDrive.setPower(v3);     //motor1
            robot.backRightDrive.setPower(v1);    //motor

            telemetry.addData("sticks", "%.2f , %.2f, %.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, angle);

            if (gamepad1.left_bumper) {
                robot.pan.setPosition(0);
                clawPosition += clawARM_SPEED;//this code here actually sets the position of the servo so it moves
            } else if (gamepad1.right_bumper) {
                robot.pan.setPosition(0.42);
            }



//            if (gamepad2.dpad_up) { //baka
//                robot.tilt.setTargetPosition(0);
//                robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.tilt.setPower(-1);
//                robot.twistClaw.setPosition(0.25);
//                robot.lift.setTargetPosition(1700);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.lift.setPower(1);//this code here actually sets the position of the servo so it moves
//            }
//
//            if (gamepad2.dpad_down) { //if the "y" button is pressed then do the next line of code
//                twistPosition -= twistARM_SPEED;   //substract from the servo position so it moves in the other direction
//                robot.twistClaw.setPosition(0.4); // PU from pit. this code here actually sets the position of the servo so it moves
//            }
//            if (gamepad2.dpad_right) { //if the "y" button is pressed then do the next line of code
//                robot.pan.setPosition(0);  // PU Samples from observatory. this code here actually sets the position of the servo so it moves
//            }
//            if (gamepad2.dpad_left) {
//                //robot.twistClaw.setPosition(0);  //this code here actually sets the position of the servo so it moves
//                // sleep(500);
//                robot.pan.setPosition(.5);  //this code here actually sets the position of the servo so it moves
//                //  panPosition += clawARM_SPEED; // a position os it movesdd to the servo
//
//            } else if (gamepad2.x) {
//                robot.tilt.setTargetPosition(-5000);
//                robot.tilt.setPower(-1);
//            } else if (gamepad2.b) {
//                robot.tilt.setTargetPosition(7000);
//                robot.tilt.setPower(1);
//            } else {
//                robot.tilt.setPower(0);
//            }


//$$$$$
            //

            if (gamepad2.dpad_left) {
                robot.pan.setPosition(0.21);
                clawPosition += clawARM_SPEED;//this code here actually sets the position of the servo so it moves
            } else if (gamepad2.dpad_right) {
                robot.pan.setPosition(0.75);
            } else if (gamepad2.left_bumper) {   //For wall grab pre-set
                robot.pan.setPosition(0.75);
            } else if (gamepad2.dpad_down) { //For middle grabbing pre-set
                robot.pan.setPosition(0.75);
            } else if (gamepad2.dpad_up) {  //Bar Hang clip pre-set
                robot.pan.setPosition(0.75);
            } else if (gamepad2.left_stick_button) { //middle grab preset
                robot.pan.setPosition(0.75);
            }

            //twist
            if (gamepad2.dpad_up) {
                robot.twistClaw.setPosition(0.30);  //this code here actually sets the position of the servo so it moves

            } else if (gamepad2.left_bumper) {
                robot.twistClaw.setPosition(0);
            } else if (gamepad2.dpad_down) {
                robot.twistClaw.setPosition(0.15);
            } else if (gamepad2.left_stick_button) {    //middle grabbing preset
                robot.twistClaw.setPosition(0.36);
            }

            //claw open close
            if (gamepad2.right_trigger > 0.5) { // if the "b" is pressed on gamepad do this next line of code
                robot.claw.setPosition(0.37);  //this code here actually sets the position of the servo so it moves
                clawPosition += clawARM_SPEED; // a position os it movesdd to the servo
            } else if (gamepad2.left_trigger > 0.5) { // if the "b" is pressed on gamepad do this next line of code
                robot.claw.setPosition(0);
                clawPosition -= clawARM_SPEED; // a position os it movesdd to the servo
                // a position os it movesdd to the servo
            } else if (gamepad2.dpad_up) {  //Bar Hang Clip Pre-set
                robot.claw.setPosition(0.37);
                clawPosition += clawARM_SPEED;
            } else if (gamepad2.left_bumper) {  //Wall Grab Pre-set
                robot.claw.setPosition(0);
                clawPosition -= clawARM_SPEED;
            } else if (gamepad2.left_stick_button) {    //middle grabbing preset
                robot.claw.setPosition(0.37);
                clawPosition += clawARM_SPEED;
            }

            //lift
            if (gamepad2.a) {
                robot.lift.setTargetPosition(50);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1.0); //this code here actually sets the position of the servo so it moves
            } else if (gamepad2.y) {
                robot.lift.setTargetPosition(1000);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1.0);
            } else if (gamepad2.right_bumper) {
                robot.lift.setTargetPosition(1700);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1.0);
            } else if (gamepad2.dpad_up) { //Hook Pre-Set
                robot.lift.setTargetPosition(1700);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1.0);
            } else if (gamepad2.left_bumper) { //Wall Pre-Set
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1.0);
            }else if (gamepad2.dpad_down) { //Wall Pre-Set
                robot.lift.setTargetPosition(3000);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1.0);
            } else if (gamepad2.left_stick_button) {    //middle grabbing preset
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1.0);
            } else {
                robot.lift.setPower(0);
            }


//tilt
            if (gamepad2.x) {
                robot.tilt.setTargetPosition(-5000);
                robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.tilt.setPower(-1);
            } else if (gamepad2.b) {
                robot.tilt.setTargetPosition(7000);
                robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.tilt.setPower(1);
                //  } else if (gamepad2.dpad_down) {
                //     robot.tilt.setTargetPosition(-650);
                //    robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //    robot.tilt.setPower(-1);
            } else if (gamepad2.dpad_up) {
                robot.tilt.setTargetPosition(0);
                robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.tilt.setPower(1);
            } else if (gamepad2.left_bumper){
                robot.tilt.setTargetPosition(1500);
                robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.tilt.setPower(1);
            } else if (gamepad2.left_stick_button) {    //middle grabbing preset
                robot.tilt.setTargetPosition(1500);
                robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.tilt.setPower(1);

            } else {
                robot.tilt.setPower(0);
            }








            //    dronePosition = Range.clip(dronePosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE); // make sure the position is valid

//            if (gamepad2.y) { // if the "a" is pressed on gamepad do this next line of code
//                robot.lift.setTargetPosition(1000);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.lift.setPower(1);  //this code here actually sets the position of the servo so it moves
//            }
//
//            if(gamepad2.left_bumper) {
//                robot.twistClaw.setPosition(0);
//                robot.claw.setPosition(0); //0 is open, .35 is closed
//
//                robot.tilt.setTargetPosition(1500);
//                robot.tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.tilt.setPower(1);
//                robot.lift.setTargetPosition(0);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.lift.setPower(1);//this code here actually sets the position of the servo so it moves
//            }
//
//            if (gamepad2.right_bumper) { // if the "a" is pressed on gamepad do this next line of code
//
//                robot.lift.setTargetPosition(1700);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.lift.setPower(1);//this code here actually sets the position of the servo so it moves
//            }
//
//            if (gamepad2.a) {
//                //this code here actually sets the position of the servo so it moves
//                robot.lift.setTargetPosition(30);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.lift.setPower(1.0);
//            }
//
//            if (gamepad2.right_trigger > 0.5) { // if the "b" is pressed on gamepad do this next line of code
//                //robot.twistClaw.setPosition(0);  //this code here actually sets the position of the servo so it moves
//                // sleep(500);
//                robot.claw.setPosition(.3);  //this code here actually sets the position of the servo so it moves
//                clawPosition += clawARM_SPEED; // a position os it movesdd to the servo
//
//            }
//            if (gamepad2.left_trigger > 0.5) { // if the "b" is pressed on gamepad do this next line of code
//                robot.claw.setPosition(0);  //this code here actually sets the position of the servo so it moves
//                clawPosition -= clawARM_SPEED; // a position os it movesdd to the servo
//            }

            telemetry.addData("pan", "%.2f", panPosition);
            telemetry.addData("twist", "%.2f", twistPosition);
            telemetry.addData("claw", "%.2f", clawPosition);
            telemetry.update();
        }
    }
}
