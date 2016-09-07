package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareNNDemobot
{
    /* Public OpMode members. */
    public DcMotor  leftFMotor   = null;
    public DcMotor  rightFMotor  = null;
    public DcMotor  leftRMotor   = null;
    public DcMotor  rightRMotor  = null;
    public Servo    armLeft         = null;
    public Servo    clawLeft        = null;
    public Servo    armRight         = null;
    public Servo    clawRight        = null;

    public final static double ARM_HOME = 0.2;
    public final static double CLAW_HOME = 0.2;
    public final static double ARM_MIN_RANGE  = 0.20;
    public final static double ARM_MAX_RANGE  = 0.90;
    public final static double CLAW_MIN_RANGE  = 0.20;
    public final static double CLAW_MAX_RANGE  = 0.7;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareNNDemobot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFMotor   = hwMap.dcMotor.get("Fleft");
        rightFMotor  = hwMap.dcMotor.get("Fright");
        leftRMotor   = hwMap.dcMotor.get("Rleft");
        rightRMotor  = hwMap.dcMotor.get("Rright");
        leftFMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFMotor.setPower(0);
        rightFMotor.setPower(0);
        leftRMotor.setPower(0);
        rightRMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        armLeft = hwMap.servo.get("Larm");
        clawLeft = hwMap.servo.get("Lclaw");
        armRight = hwMap.servo.get("Rarm");
        clawRight = hwMap.servo.get("Rclaw");
        armLeft.setPosition(ARM_HOME);
        clawLeft.setPosition(CLAW_HOME);
        armRight.setPosition(ARM_HOME);
        clawRight.setPosition(CLAW_HOME);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
