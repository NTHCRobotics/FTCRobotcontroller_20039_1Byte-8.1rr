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

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove ~or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tic Tac", group="Isaiah")
//@Disabled  This way it will run on the robot
public class tickcounter extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();  //timer

    /*
    Declare motors t==o type DcMotorEx
    Documentation:
    https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
     */
    private DcMotorEx lazyS;
    private DcMotorEx intake;
    private DcMotorEx arm;
    private Servo bucket;


    private DcMotorEx motorArm;
    private double maxTicsPerSec = 3000;
    private int targetPosition;
    private String buttonPressed;

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MIN_POS = 0.0;     // Maximum rotational position
    static final double MAX_POS = 1.0;     // Minimum rotational position\
    private double curTicPos = 0;
    private double ticMultiplier = 1;

    double position = 0.0; // Start at bottom position
    boolean rampUp = true;

    private int level = 1; //1 = ground, 2 = level 2, 3 = level 3
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lazyS = hardwareMap.get(DcMotorEx.class, "lazyS");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        bucket = hardwareMap.get(Servo.class,"bucket");

        motorArm = hardwareMap.get(DcMotorEx.class, "arm");
        motorArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        this.targetPosition = 0;
        this.buttonPressed = "";



       /*
           Set up motors so they run without the encoders
           This way they run freely.  They won't go to a specific position or count the number of rotations
           It will now run with range from -1.0 to 1.0
           See Documentation for other encoder modes
           https://docs.revrobotics.com/rev-control-system/programming/using-encoder-feedback
        */
        lazyS.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        position = 1.0;


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //mecanum wheel code
        //https://ftccats.github.io/software/ProgrammingMecanumWheels.html

        double lazyPower = 0.0;
        double spinnerPower = 0.0;
        double armPower = 0.0;


        //Go
        if (gamepad1.right_bumper) { //Moves set motor forward 1 tic times how fast you chose
            maxTicsPerSec = (1 * ticMultiplier);
            curTicPos += maxTicsPerSec;
            telemetry.addData("Current position: ", "%.1f tics", curTicPos);
            telemetry.addData("Tic Multiplier", "%.1f", ticMultiplier);
            telemetry.update();
        }
        else if (gamepad1.left_bumper){ //Moves set motor back 1 tic times how fast you chose
            maxTicsPerSec = (-1 * ticMultiplier);
            curTicPos += maxTicsPerSec;
            telemetry.addData("Current position: ", "%.1f tics", curTicPos);
            telemetry.addData("Tic Multiplier", "%.1f", ticMultiplier);
            telemetry.update();
        }

        if (gamepad1.dpad_right){ //multiplies speed by 5 times
            ticMultiplier *= 5;
        }
        else if (gamepad1.dpad_left) { //slows speed by 5 times
            ticMultiplier /= 5;
        }

        telemetry.addData("position of servo", "%.1f", position);



        //change the power for each wheel



        lazyS.setPower(lazyPower);
        intake.setPower(spinnerPower);
        arm.setPower(armPower);
        bucket.setPosition(position);


        /*

        PLAYER 2 GAMEPAD


         */
        //get input


//        if



            /*
                Keep the target position >= 0
                Unless dpad_down is pressed
             */
        if (arm.getCurrentPosition() == 0 && gamepad2.dpad_down){
            targetPosition -= 10;
        }

            /*
                check if magnet is touching sensor
                True = NOT touching sensor
             */



        //set motor
        //motorArm.setTargetPosition(targetPosition);
        motorArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorArm.setVelocity(maxTicsPerSec);
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public boolean distance(int cm) //cm = how far away to check
    {
        return true;
    }
}


