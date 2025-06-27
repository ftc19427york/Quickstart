package pedroPathing.examples;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class OldHardwareMecanum
{

    private final ElapsedTime runtime = new ElapsedTime();
    public static double kpH = 0.075, kdH = 0.001, kpX = 0.11, kdX = 0.7, kpY = 0.11, kdY = 0.7;

    Pose2D currentPose;

    double oldTime = 0;

    double errorX, errorY, errorH;
    double lastErrorH = 0, lastErrorX = 0, lastErrorY = 0;

    double correctionH, correctionX, correctionY;

    int currentL = 0, lastL = 0;

    /* Public OpMode members. */
 //   public wheels drivetrain;
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive     = null;
    public DcMotor  backRightDrive     = null;
    public DcMotor  lift     = null;
    public DcMotor  tilt     = null;
    public Servo pan = null;
    public Servo claw = null;
    public Servo twistClaw = null;
   // GoBildaPinpointDriver odo;

    //  public DistanceSensor rangeFinder = null;
    //  public TouchSensor toucher = null;
    //  public RevColorSensorV3 colorSensor = null;

    public final static double panARM_HOME = 0.75; // Starting point for Servo Arm 0.75
    public final static double clawARM_HOME = 0.37; // Starting point for Servo Arm

    public final static double twistClawARM_HOME = 0.6; // Starting point for Servo Arm **twisting of the entire claw assembly

    public final static double droneARM_MIN_RANGE = 0.0;  //Smallest number value allowed for servo position
    public final static double droneARM_MAX_RANGE = 1.0;  //Largest number allowed for servo position


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public OldHardwareMecanum(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;
       // odo = hwMap.get(GoBildaPinpointDriver.class,"odo");
        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "motor3");
        frontRightDrive = hwMap.get(DcMotor.class, "motor2");
        backLeftDrive    = hwMap.get(DcMotor.class, "motor1");
        backRightDrive    = hwMap.get(DcMotor.class, "motor");

      //  drivetrain = new wheels(frontLeftDrive, frontRightDrive);
     //   drivetrain.hello();

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // motor3
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);// motor2
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE); // motor1
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);// motor

        lift    = hwMap.get(DcMotor.class, "motor4");
        tilt    = hwMap.get(DcMotor.class, "motor5");

        lift.setDirection(DcMotor.Direction.FORWARD); // motor4
        tilt.setDirection(DcMotor.Direction.FORWARD);// motor5
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        this.stopDriving();



        // Define and initialize ALL installed servos
        pan=hwMap.servo.get("servo2"); // set equal to name of the servo motor in DS
        pan.setPosition(panARM_HOME); //setPosition actually sets the servo's position and move it
        claw=hwMap.servo.get("servo1"); // set equal to name of the servo motor in DS
        claw.setPosition(clawARM_HOME); //setPosition actually sets the servo's position and move it
        twistClaw=hwMap.servo.get("servo"); // set equal to name of the servo motor in DS
        twistClaw.setPosition(twistClawARM_HOME); //setPosition actually sets the servo's position and move it


        //     rangeFinder = hwMap.get(DistanceSensor.class, "sensor_distance");
        //    toucher = hwMap.get(TouchSensor.class, "touch");
        //   frontColor = hwMap.get(RevColorSensorV3.class, "sensor_color");
    }
    public void driveWithoutEncode() {
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveWithEncode() {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stopResetEncode() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runToPosition() {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopDriving() {
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void driveToPoint (double x, double y, double heading, double timeOut) {
        Pose2D desiredPose = new Pose2D (DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
        updateOptometryData();
        errorX = desiredPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        errorY = desiredPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
        errorH = desiredPose.getHeading(AngleUnit.DEGREES) - currentPose.getHeading(AngleUnit.DEGREES);

        runtime.reset();

        while ((Math.abs(errorX) > 0.5 || Math.abs(errorY) > 0.5 || Math.abs(errorH) > 1) && runtime.seconds() < timeOut){
            updateOptometryData();
            //find error from current position
            errorX = desiredPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
            errorY = desiredPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
            errorH = desiredPose.getHeading(AngleUnit.DEGREES) - currentPose.getHeading(AngleUnit.DEGREES);
            while(Math.abs(errorH) > 180) {
                if(errorH > 180) {
                    errorH -= 360;
                }

                if(errorH < -180) {
                    errorH += 360;
                }
            }

            //move robot | increase gains to get better accuracy
            correctionH = kpH*errorH + kdH*(errorH-lastErrorH);
            correctionX = kpX*errorX + kdX*(errorX-lastErrorX);
            correctionY = kpY*errorY + kdY*(errorY-lastErrorY);
            drive(-correctionY, -correctionX, -correctionH);
            lastErrorH = errorH;
            lastErrorX = errorX;
            lastErrorY = errorY;
        }
    }

    public void drive(double x, double y, double theta) {

        /**gets squared values from the driver's stick input**/
        double r = Math.hypot(-x, -y);
        /**finds the desired angle that the driver wants to move the robot**/
        double robotAngle = Math.atan2(-y, x) - Math.PI / 4;
        /**sets the movement angle by finding the difference of the robots angle, the input angle and the offset value
         * the offset value is set by the the driver if the imu does not reset after auto*/
        robotAngle = robotAngle - Math.toRadians(currentPose.getHeading(AngleUnit.DEGREES));

        double v1 = r * Math.cos(robotAngle) + theta;
        double v2 = r * Math.sin(robotAngle) - theta;
        double v3 = r * Math.sin(robotAngle) + theta;
        double v4 = r * Math.cos(robotAngle) - theta;

        // Output the safe vales to the motor drives.++
        v1 = Range.clip(v1, -0.9, 0.9);
        v2 = Range.clip(v2, -0.9, 0.9);
        v3 = Range.clip(v3, -0.9, 0.9);
        v4 = Range.clip(v4, -0.9, 0.9);
        frontLeftDrive.setPower(v1);    //motor3
        backRightDrive.setPower(v4);    //motor
        frontRightDrive.setPower(v2);   //motor2
        backLeftDrive.setPower(v3);     //motor1
    }

    public void updateOptometryData() {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
     //   odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
        //odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             *


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
      //  currentPose = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH), currentPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
    //    Pose2D vel = odo.getVelocity();
    //    String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.DEGREES));
   //     telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
      //  telemetry.addData("Status", odo.getDeviceStatus());

    //    telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
    }

}


