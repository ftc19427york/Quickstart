package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "Example Field-Centric Teleop", group = "Examples")
public class ExampleFieldCentricTeleop extends OpMode {
    HardwareMecanum robot = new HardwareMecanum(); //reference the hardware file name

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
        robot.init(hardwareMap);

    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: false
        */

        //twist claw
        if (gamepad2.dpad_left) { //twist 90
            robot.twistClaw.setPosition(0.21);
            robot.twistPosition += robot.twistARM_SPEED;//this code here actually sets the position of the servo so it moves
        } else if (gamepad2.dpad_right) { //twist to 0
            robot.twistClaw.setPosition(0.75);
        } else if (gamepad2.left_bumper) {   //For wall grab pre-set
            robot.twistClaw.setPosition(0.75);
        } else if (gamepad2.dpad_down) { //lift extends to high basket
            robot.twistClaw.setPosition(0.75);
        } else if (gamepad2.dpad_up) {  //high chamber scoring pre-set
            robot.twistClaw.setPosition(0.75);
        } else if (gamepad2.left_stick_button) { //preset to grab from submersible
            robot.twistClaw.setPosition(0.75);
        }

        //pan entire claw
        if (gamepad2.dpad_up) { //high chamber scoring pre-set
            robot.panClaw.setPosition(0.30);
            robot.panPosition += robot.panARM_SPEED;//this code here actually sets the position of the servo so it moves
        } else if (gamepad2.left_bumper) { //wall grab preset
            robot.panClaw.setPosition(0.15);
        } else if (gamepad2.dpad_down) { //lift extends to high basket
            robot.panClaw.setPosition(0.15);
        } else if (gamepad2.left_stick_button) {    //preset to grab from submersible
            robot.panClaw.setPosition(0.36);
        }

        //claw open close
        if (gamepad2.right_trigger > 0.5) { // closes claw
            robot.claw.setPosition(0.37);  //this code here actually sets the position of the servo so it moves
            robot.clawPosition += robot.clawARM_SPEED; // a position os it movesdd to the servo
        } else if (gamepad2.left_trigger > 0.5) { // opens claw
            robot.claw.setPosition(0);
            robot.clawPosition -=  robot.clawARM_SPEED; // a position os it movesdd to the servo
            // a position os it movesdd to the servo
        } else if (gamepad2.dpad_up) {  //high chamber scoring pre-set
            robot.claw.setPosition(0.37);
            robot.clawPosition +=  robot.clawARM_SPEED;
        } else if (gamepad2.left_bumper) {  //Wall Grab Pre-set
            robot.claw.setPosition(0);
            robot.clawPosition -=  robot.clawARM_SPEED;
        } else if (gamepad2.left_stick_button) {   //preset to grab from submersible
            robot.claw.setPosition(0.37);
            robot.clawPosition +=  robot.clawARM_SPEED;
        }

        //tilt
        if (gamepad2.x) { //lift tilt up
            robot.setTiltPosition(-600, 0.9);//390
        } else if (gamepad2.b) { //lift tilt down
            robot.setTiltPosition(840, 0.9);//600
        } else if (gamepad2.dpad_up) { //high chamber scoring pre-set
            robot.setTiltPosition(0, 0.9);
        } else if (gamepad2.left_bumper){ //wall grab preset
            robot.setTiltPosition(280, 0.9);  //1500  375
        } else if (gamepad2.left_stick_button) {  //preset to grab from submersible
            robot.setTiltPosition(290, 0.9);
        } else {
            robot.tilt.setPower(0);
        }

        //lift
        if (gamepad2.a) { //lift position 0
            robot.setLiftPosition(0, 0.9);
        } else if (gamepad2.y) { //extends halfway
            robot.setLiftPosition(730, 0.9);
        } else if (gamepad2.right_bumper) { //extends lift to boundary
            robot.setLiftPosition(1241, 0.9);
        } else if (gamepad2.dpad_up) {                   //high chamber scoring pre-set
            robot.setLiftPosition(1600, 0.9);
        } else if (gamepad2.left_bumper) {                   //Wall grab Pre-Set
            robot.setLiftPosition(0, 0.9);
        }else if (gamepad2.dpad_down) { //lift extends to high basket
            robot.setLiftPosition(2200, 0.9);
        } else if (gamepad2.left_stick_button) {             //preset to grab from submersible
            robot.setLiftPosition(0, 0.9);
        } else {
            robot.lift.setPower(0);
        }

        //extras on driver controller
        if (gamepad1.left_bumper) { //wall grab preset
            robot.twistClaw.setPosition(0);
            robot.twistPosition += robot.twistARM_SPEED;//this code here actually sets the position of the servo so it moves
        } else if (gamepad1.right_bumper) {  //extends lift to boundary
            robot.twistClaw.setPosition(0.42);
        }

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Tilt Position", robot.tilt.getCurrentPosition());
        telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
        telemetry.addData("Claw Position", robot.claw.getPosition());  //Open/close claw
        telemetry.addData("Twist Position", robot.twistClaw.getPosition());  //Twist purple claw
        telemetry.addData("Pan Position", robot.panClaw.getPosition());  //Paw the entire claw
        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}