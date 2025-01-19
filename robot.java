//Imports
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="AutoMode (Score Specimen)", group="Linear OpMode")
public class Auto_Score_Specimen_Middle extends LinearOpMode {
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor armMotor = null;
    private CRServo intake = null;
    private Servo wristServo = null;
    private DcMotor slides = null;
    private IMU imu = null;
    
    private double headingError = 0;
    
    
    //drivetrain IamYou
    
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double strafeDriveSpeed = 0;
    private double strafeTurnSpeed = 0;
    private double strafeSpeed = 0;
    private double frontLeftSpeed = 0;
    private double frontRightSpeed = 0;//
    private double backLeftSpeed = 0;////
    private double backRightSpeed = 0;///``
    private int frontLeftTarget = 0;/////
    private int frontRightTarget = 0;////
    private int backLeftTarget = 0;//////
    private int backRightTarget = 0;/////
    
    static final double DRIVE_SPEED = 0.4;////////
    static final double TURN_SPEED = 0.2;/////////
    static final double STRAFE_SPEED = 0.2;/////////
    static final double HEADING_THRESHOLD = 1.0;//


    // this is ALMOST the best code
    private static final double TICKS_PER_REV = 28 * (((1+(46.0/17)))*(1+(46.0/11))); // NEED TO CHANGE
    private static final double WHEEL_DIAMETER_MILLIMETER = 104; // Diameter of the wheel in millimeter
    private static final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_MILLIMETER; // Circumference
    private static final double TICKS_PER_MILLIMETER = TICKS_PER_REV / CIRCUMFERENCE;

    final double SLIDER_TICKS_PER_ROTATION = 383.6;
    final double SLIDE_TICKS_PER_MM = SLIDER_TICKS_PER_ROTATION/120; //3.19
    
    final double SLIDE_OUT = 3.55 * (SLIDER_TICKS_PER_ROTATION); //change this a full extentions is 1342.6 units
    final double SLIDE_IN = 0.5 * (SLIDER_TICKS_PER_ROTATION);
    
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more respo
    static final double     P_STRAFE_GAIN           = 0.03; 
    //arm
    final double ARM_TICKS_PER_DEGREE =
            28 * 250047.0 / 4913.0 * 100.0 / 20.0 * (1.0 / 360.0); // Ticks per degree

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 80 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_HIGH  = 97 * ARM_TICKS_PER_DEGREE;
    
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8; //0.675
    final double WRIST_FOLDED_OUT  = 0.5; // 1
    final double WRIST_FOLDED_BACK = 0.2;
    

    private ElapsedTime runtime = new ElapsedTime();

    private void stopMotorPower() {
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);///
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);////
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);///
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d:%7d:%7d",      frontLeftTarget,  frontRightTarget, backLeftTarget, backRightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d:%7d:%7d",      frontLeftDrive.getCurrentPosition(),
            frontRightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f : %5.2f : %5.2f", frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
        telemetry.update();
    }

    // Best function ever
    public void turnToHeading(double maxTurnSpeed, double heading) {
        
        
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
    
   



    
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    
    
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading; 
        
        
        headingError=targetHeading - getHeading();
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180){
            headingError -= 360;
        }
        while (headingError <= -180){
            headingError += 360;
        }
// moveRobotStrafe(maxdrivespeed)
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double forward, double turn) {
        
        driveSpeed = forward;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;

        frontLeftSpeed  = forward - turn;
        frontRightSpeed = forward + turn;
        backLeftSpeed  = forward - turn;
        backRightSpeed = forward + turn;
        
        double max1 = Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed));
        double max2 = Math.max(max1, Math.abs(frontLeftSpeed));
        double max = Math.max(max2, Math.abs(frontRightSpeed));
        
        if (max > 1.0){
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }
        telemetry.addData("Move Robot Wheel Speeds L : R", "%5.2f : %5.2f : %5.2f : %5.2f", frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
        telemetry.update();
        
        frontLeftDrive.setPower(frontLeftSpeed);
        frontRightDrive.setPower(frontRightSpeed);
        backLeftDrive.setPower(backLeftSpeed);
        backRightDrive.setPower(backRightSpeed);
    }

    // moverobotstrage
    public void moveRobotStrafe(double strafe, double turn) {

        frontLeftSpeed  =  strafe + turn;
        frontRightSpeed =  - strafe - turn;
        backLeftSpeed  =  - strafe + turn;
        backRightSpeed =  strafe - turn;
        
        double max1 = Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed));
        double max2 = Math.max(max1, Math.abs(frontLeftSpeed));
        double max = Math.max(max2, Math.abs(frontRightSpeed));
        
        if (max > 1.0){
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }
        frontLeftDrive.setPower(frontLeftSpeed);
        frontRightDrive.setPower(frontRightSpeed);
        backLeftDrive.setPower(backLeftSpeed);
        backRightDrive.setPower(backRightSpeed);
        telemetry.addData("Move Robot Wheel Speeds L : R", "%5.2f : %5.2f : %5.2f : %5.2f", frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
        telemetry.update();
    }
    
    public void slidePosition(double s){
        if(s == 1){
            slides.setTargetPosition((int) (SLIDE_IN));
        } else {
            slides.setTargetPosition((int) (SLIDE_OUT));
        }
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(2100);
        while(opModeIsActive() && slides.isBusy()){
        }
    }
    
    public void driveStrafe(double maxDriveSpeed, double distance, double heading, double direction){
        if(opModeIsActive()){
            if(direction == 1){
                int moveCounts = (int)(distance * TICKS_PER_MILLIMETER);
                frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
                frontRightTarget = frontRightDrive.getCurrentPosition() - moveCounts;
                backLeftTarget = backLeftDrive.getCurrentPosition() - moveCounts;
                backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;
            } else {
                int moveCounts = (int)(distance * TICKS_PER_MILLIMETER);
                frontLeftTarget = frontLeftDrive.getCurrentPosition() - moveCounts;
                frontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
                backLeftTarget = backLeftDrive.getCurrentPosition() + moveCounts;
                backRightTarget = backRightDrive.getCurrentPosition() - moveCounts;
            }
            
            frontLeftDrive.setTargetPosition(frontLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            if(direction == 1){
                moveRobotStrafe(maxDriveSpeed, 0);
            } else {
                moveRobotStrafe(-maxDriveSpeed, 0);
            }
            
            while (opModeIsActive() && (backLeftDrive.isBusy() && backRightDrive.isBusy() && frontLeftDrive.isBusy() && frontRightDrive.isBusy())){
    
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0){
                    turnSpeed *= -1;
                }
                // Apply the turning correction to the current driving speed.
                if(direction == 1){
                    moveRobotStrafe(maxDriveSpeed, 0);
                } else {
                    moveRobotStrafe(-maxDriveSpeed, 0);
                }
        
            }
            
            moveRobot(0, 0);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        }
    }    
    
    public void driveStraight(double maxDriveSpeed, double distance, double heading){
        // Ensure that the OpMode is still active
        if(opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int moveCounts = (int)(distance * TICKS_PER_MILLIMETER);
            frontLeftTarget =  moveCounts;
            frontRightTarget =  moveCounts;
            backLeftTarget =  moveCounts;
            backRightTarget =  moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeftDrive.setTargetPosition(frontLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);
            // telemetry.addData("Motion", maxDriveSpeed);
            // telemetry.update();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (backLeftDrive.isBusy() && backRightDrive.isBusy() && frontLeftDrive.isBusy() && frontRightDrive.isBusy())){
    
                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0){
                    turnSpeed *= -1;
                }
                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);
                sendTelemetry(true);
        
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

        
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
     
    private void intake(int mode) {
        if (mode == -1) {
            intake.setPower(INTAKE_COLLECT);
        } else if (mode == 1) {
            intake.setPower(INTAKE_DEPOSIT);
        } else {
            intake.setPower(INTAKE_OFF);
        }
    }

    private void armPosition(double position, double fudge, double power) {
        armMotor.setTargetPosition((int) (position + (fudge * ARM_TICKS_PER_DEGREE)));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
        while(opModeIsActive() && armMotor.isBusy()){
        }
    }

    private void wrist(int mode) {
        if (mode == 1) {
            wristServo.setPosition(WRIST_FOLDED_IN);
        } else if (mode == 2){
            wristServo.setPosition(WRIST_FOLDED_OUT);
        } else {
            wristServo.setPosition(WRIST_FOLDED_BACK);
        }
    }
    
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intake = hardwareMap.get(CRServo.class, "intakeServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        slides = hardwareMap.get(DcMotor.class, "slides");
        
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        slides.setDirection(DcMotor.Direction.REVERSE);
        //IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();
        // Wait for the game to start
        waitForStart();
        runtime.reset();
        intake(-1);
        armPosition(ARM_CLEAR_BARRIER+(10*ARM_TICKS_PER_DEGREE), 0, 2100);
        slidePosition(1);
        wrist(0);
        armPosition(ARM_SCORE_SPECIMEN+(0*ARM_TICKS_PER_DEGREE), 0, 2100);
        driveStraight(0.5,-700,0);
        
        sleep(1000);
        armPosition(ARM_SCORE_SPECIMEN+(-2*ARM_TICKS_PER_DEGREE), 0, 2100);
        driveStraight(0.2,300,0);
        intake(1);
        sleep(1000);
        driveStraight(0.2,200,0);
        slidePosition(1);
        wrist(2);
        armPosition(ARM_CLEAR_BARRIER+(0*ARM_TICKS_PER_DEGREE), 0, 500);
        intake(0);
        
        // retract the slide fully and wait to retract
        slides.setTargetPosition((int) (0));
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(2100);
        while(opModeIsActive() && slides.isBusy()){
        }
        
        
        turnToHeading(0.4, -90);
        driveStrafe(0.3,350,-90,-1);
        wrist(2);
        intake(-1);
        armPosition(ARM_CLEAR_BARRIER+(0*ARM_TICKS_PER_DEGREE), 0, 2100);
        slides.setTargetPosition((int)(SLIDER_TICKS_PER_ROTATION*1.5));
        
        driveStraight(0.3,-800,-90);
        sleep(1000);
        
        armPosition(ARM_CLEAR_BARRIER+(10*ARM_TICKS_PER_DEGREE), 0, 2100);
        driveStrafe(0.3,350,-90,1);
        wrist(0);
        driveStraight(0.7,950,-90);
        turnToHeading(0.4, 0);
        
        slides.setTargetPosition(1);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(2100);
        while(opModeIsActive() && slides.isBusy()){
        }
        
        
        armPosition(ARM_SCORE_SPECIMEN+(4*ARM_TICKS_PER_DEGREE), 0, 2100);
        
        driveStraight(0.5, -525, 0);
        armPosition(ARM_SCORE_SPECIMEN+(2*ARM_TICKS_PER_DEGREE), 0, 2100);
        sleep(1000);
        driveStraight(0.2,300,0);
        intake(1);
        sleep(1000);
        driveStraight(0.2,200,0);
        slidePosition(1);
        wrist(2);
        armPosition(ARM_CLEAR_BARRIER+(0*ARM_TICKS_PER_DEGREE), 0, 1500);
        intake(0);
        
        // retract the slide fully and wait to retract
        slides.setTargetPosition((int) (0));
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(2100);
        driveStrafe(1, 1000, 0, -1);
        
        
        
        
        
        // driveStraight(0.5,-1000,0);
        // driveStrafe(0.5,300,0,-1);
        // driveStraight(0.5,1100,0);
        // driveStraight(0.5,-1100,0);
        // driveStrafe(0.5,300,0,-1);
        // driveStraight(0.5,1100,0);
        // driveStraight(0.5,-300,0);
        // sleep(2000);
        // driveStraight(0.5,300,0);
        
        
        
        
    
        sleep(10000);
    }
}

//857 millimeters from submersable to back of piece


