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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
import static org.firstinspires.ftc.teamcode.HardwarePushbot.armServo;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
public class TeleOp_Drive_Pov extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() {
        //creating variables
        double left;
        double right;
        double drive;
        double turn;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            * In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            * This way it's also easy to just drive straight, or just turn.
            */

            //Left stick y axis moves it forward and back, x axis moves it left and right. If driving isn't preferred, change the second line to 'gamepad1.right_stick_x;'
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.left_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.backLeft.setPower(left);
            robot.frontLeft.setPower(left);
            robot.backRight.setPower(right);
            robot.frontRight.setPower(right);


            /* Makes the arm move up and down according to the toggles on the second gamepad
            *  Uses servos armServo and armServo2 (preferably working in sync but we'll see. armServo2 can be free moving if worst comes to worst)
            */
            //TBA
            //Motor not in place


            /* Makes the claw open and close according to the x and y buttons on the second gamepad
            * x = open, y=close
            *  Uses servo clawServo
            */
            //TBT
            //Change the angles if it goes too far or messes something up
            if (gamepad2.x)
                robot.clawServo.setPosition(1);
            else if (gamepad2.y)
                robot.clawServo.setPosition(0);



            /* Makes the motor move to pull back the string and release it according to the a and b buttons on the second gamepad
            *  a = tighten, b = release
            * Uses motor release
             */
            //TBT
            //swap forward and reverse is a doesn't tighten and b doesn't release
            while (gamepad2.a) {
                robot.release.setDirection(DcMotor.Direction.FORWARD);
                robot.release.setPower(0.45);
            }
            while (gamepad2.b) {
                robot.release.setDirection(DcMotor.Direction.REVERSE);
                robot.release.setPower(0.45);
            }






            /* Makes the servo fall to hold the elastic and rise to shoot according to Lb and Rb on the second gamepad
            *  Lb = hold,  Rb = shoot
            *  Uses servo triggerServo (to be defined)
             */
            //TBA
            //Servo not in place


            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
