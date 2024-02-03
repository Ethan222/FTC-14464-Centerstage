package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Motor;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeRotator;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {
    private static boolean singleDriverMode = false;
    public static double WHEEL_SLOW_SPEED = .3;
    public static final double ARM_SLOW_SPEED = .2;
    private static Pose2d startPose = new Pose2d();
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.setPoseEstimate(startPose);

        ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
        ElapsedTime wheelTimer = new ElapsedTime(), loopTimer = new ElapsedTime();

        int direction = 1;
        double speed = 1, heading;
        boolean usingRoadrunner = false;

        Gamepad gamepad;

        while(opModeInInit() && !(gamepad1.start && gamepad1.back) && !(gamepad2.start && gamepad2.back)) {
            telemetry.addLine("Initialized");
            telemetry.addData("Single driver mode (back + a/b)", singleDriverMode);
            telemetry.addData("\nStart pose", "(%.1f, %.1f) @ %.1f deg", startPose.getX(), startPose.getY(), startPose.getHeading());
            telemetry.update();

            if((gamepad1.back && gamepad1.a) || (gamepad2.back && gamepad2.a))
                singleDriverMode = true;
            else if((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b))
                singleDriverMode = false;
        }

        telemetry.setAutoClear(false);
        telemetry.setMsTransmissionInterval(150);
        Telemetry.Item singleDriverModeTelemetry = telemetry.addData("Single driver mode", singleDriverMode);
        Telemetry.Item directionTelemetry = telemetry.addData("Direction (RB/LB)", "forward (INTAKE)");
        Telemetry.Item speedTelemetry = telemetry.addData("Wheel speed (RT)", speed);
        Telemetry.Item headingTelemetry = telemetry.addData("Heading", null);
        robot.intake.setTelemetry(telemetry.addData("\nintake", "[0.0]"));
        robot.autoClaw.setTelemetry(telemetry.addData("pixel flusher", null));
        robot.claw1.setTelemetry(telemetry.addData("\nclaw 1", null));
        robot.claw2.setTelemetry(telemetry.addData("claw 2", null));
        robot.outtake.setTelemetry(telemetry.addData("outtake", "(%d) [0.0]", robot.outtake.getPosition()));
        robot.outtake.updateTelemetry();
        robot.outtake.rotator.setTelemetry(telemetry.addData("- rotator", null));
        robot.hangSubsystem.setTelemetry(telemetry.addData("\nhang", null), telemetry.addData("- rotator", null));
        robot.hangSubsystem.updateMotorTelemetry();
        robot.launcher.setLauncherTelemetry(telemetry.addData("launcher", null));
        robot.launcher.updateLauncherTelemetry();
        robot.launcher.setRotatorTelemetry(telemetry.addData("- rotator", null));
        telemetry.addData("\nLoop time", "%.0f ms", loopTimer::milliseconds);

//        robot.intake.lower();
//        robot.launcher.rotator.goToRight();

        while (opModeIsActive() && !(gamepad1.start && gamepad1.back) && !(gamepad2.start && gamepad2.back)) {
            if(singleDriverMode && gamepad1.back)
                robot.drive.setWeightedDrivePower(new Pose2d());
            else if(!usingRoadrunner) {
                robot.drive.setWeightedDrivePower(
                        new Pose2d(
                                direction * speed * gamepad1.left_stick_y,
                                direction * speed * gamepad1.left_stick_x,
                                speed * -gamepad1.right_stick_x
                        )
                );
            }
            robot.drive.update();

            heading = robot.drive.getPoseEstimate().getHeading();
            if(heading > Math.PI)
                heading -= 2 * Math.PI;
            headingTelemetry.setValue("%.1f deg", heading * 180 / Math.PI);

            // straighten
            if(gamepad1.dpad_down && !singleDriverMode && !usingRoadrunner && wheelTimer.seconds() > 1 && Math.abs(heading) > .1) {
                usingRoadrunner = true;
                robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Pose2d startPose = robot.drive.getPoseEstimate();
                robot.drive.followTrajectoryAsync(robot.drive.trajectoryBuilder(startPose)
                        .lineToSplineHeading(new Pose2d(startPose.getX(), startPose.getY(), 0))
                        .build()
                );
                wheelTimer.reset();
            }
            if(usingRoadrunner && !robot.drive.isBusy())
                usingRoadrunner = false;
            telemetry.addLine();

            // reverse direction
            if((!singleDriverMode && gamepad1.left_bumper) || (gamepad1.start && gamepad1.left_bumper)) {
                direction = -1;
                directionTelemetry.setValue("reverse (OUTTAKE)");
            } else if((!singleDriverMode && gamepad1.right_bumper) || (gamepad1.start && gamepad1.right_bumper)) {
                direction = 1;
                directionTelemetry.setValue("forward (INTAKE)");
            }

            // slow mode
            if((!singleDriverMode && gamepad1.right_trigger > .1) || gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad1.start) {
                speed = WHEEL_SLOW_SPEED;
                speedTelemetry.setValue(speed);
            } else if(speed != 1) {
                    speed = 1;
                    speedTelemetry.setValue(1);
            }

            // single drive mode
            if((gamepad1.back && gamepad1.a) || (gamepad2.back && gamepad2.a)) {
                singleDriverMode = true;
                singleDriverModeTelemetry.setValue(true);
            } else if((gamepad1.back && gamepad1.b) || (gamepad2.back && gamepad2.b)) {
                singleDriverMode = false;
                singleDriverModeTelemetry.setValue(false);
            }

            if(singleDriverMode)
                gamepad = gamepad1;
            else
                gamepad = gamepad2;

            // intake
            // can only intake if arm is down and outtake is retracted
            if(gamepad.start && gamepad.right_trigger > 0) {
                robot.outtake.setDownPosition();
                robot.outtake.rotator.retractFully();
                robot.claw1.up();
                robot.claw2.up();
            }
            if(gamepad.right_trigger > 0 && !gamepad.back && robot.outtake.getState() == Motor.State.DOWN && robot.outtake.rotator.getState().equals(OuttakeRotator.RETRACTED))
                robot.intake.in(gamepad.right_trigger);
            else if(gamepad.left_trigger > 0 && !gamepad.back)
                robot.intake.out(gamepad.left_trigger);
            else if(robot.intake.getPower() != 0)
                robot.intake.stop();

            // raise/lower intake
            if(gamepad.back && (gamepad.right_trigger > 0 || gamepad.left_trigger > 0)) {
                if (gamepad.right_trigger > .8)
                    robot.intake.lower();
                else if (gamepad.left_trigger > .8)
                    robot.intake.raise();
                else if (gamepad.right_trigger > 0)
                    robot.intake.lower(.01);
                else
                    robot.intake.raise(.01);
                robot.intake.updateTelemetry();
            }

            // claws
            if(gamepad.a && !gamepad.start && !gamepad.back)
                robot.claw1.down();
            else if(gamepad.b && !gamepad.start && !gamepad.back)
                robot.claw1.up();
            if(gamepad.x && !gamepad.start && !gamepad.back)
                robot.claw2.down();
            else if(gamepad.y && !gamepad.start && !gamepad.back)
                robot.claw2.up();

            // raise outtake
            if((gamepad.dpad_up && !gamepad.back) || (gamepad.dpad_down && !gamepad.back))
                robot.outtake.runUsingEncoder();
            else if(gamepad2.left_stick_y != 0 || gamepad.dpad_up || gamepad.dpad_down)
                robot.outtake.runWithoutEncoder();

            if (robot.outtake.isUsingEncoder()) {
                if (gamepad.dpad_up && gamepad.start)
                    robot.outtake.setUpPosition();
                else if (gamepad.dpad_down && gamepad.start)
                    robot.outtake.setDownPosition();
                else if (gamepad.dpad_up) {
                    // when arm goes up, also rotate outtake & close claws
                    robot.claw1.down();
                    robot.claw2.down();
                    if(!robot.outtake.rotator.getState().equals(OuttakeRotator.EXTENDED))
                        robot.outtake.rotator.rotateFully();
                    robot.outtake.stopHolding();
                    executorService.schedule(() -> {
                        if(robot.outtake.getPosition() < Outtake.POSITION_1)
                            robot.outtake.goToPosition(Outtake.POSITION_1);
                        else
                            robot.outtake.goToPosition(Outtake.POSITION_2);
                    }, 500, TimeUnit.MILLISECONDS);
                } else if (gamepad.dpad_down) {
                    robot.outtake.stopHolding();
                    robot.outtake.goToDownPosition();
                    executorService.schedule(robot.outtake::stop, 2, TimeUnit.SECONDS);
                    // when arm goes down, also retract outtake
                    executorService.schedule(robot.outtake.rotator::retractFully, 800, TimeUnit.MILLISECONDS);
                } else if(robot.outtake.isIdle()) {
                    if (robot.outtake.getState() == Motor.State.DOWN) {
                        robot.outtake.stopHolding();
                        robot.outtake.stop();
                    } else
                        robot.outtake.hold();
                }
            } else {
                if(gamepad2.left_stick_y != 0) {
                    robot.outtake.stopHolding();
                    robot.outtake.setPower(-gamepad2.left_stick_y);
                } else if(gamepad.dpad_up && (robot.outtake.getState() != Motor.State.DOWN || robot.outtake.rotator.getState().equals(OuttakeRotator.EXTENDED))) {
                    robot.outtake.stopHolding();
                    robot.outtake.accelerateUp();
                } else if(gamepad.dpad_down) {
                    robot.outtake.stopHolding();
                    robot.outtake.accelerateDown();
                } else if (robot.outtake.getState().equals(Motor.State.DOWN)) {
                    robot.outtake.stopHolding();
                    robot.outtake.decelerate();
                } else
                    robot.outtake.hold();
            }
            // rotate to avoid the bar
            if((robot.outtake.getPower() > 0 && robot.outtake.getState() == Motor.State.DOWN && robot.outtake.rotator.getState().equals(OuttakeRotator.RETRACTED)) ||
                    (robot.outtake.getPower() < 0 && robot.outtake.getPosition() - robot.outtake.getDownPosition() < 400 && robot.outtake.getPosition() - robot.outtake.getDownPosition() > 200 && robot.outtake.rotator.getState().equals(OuttakeRotator.RETRACTED)))
                robot.outtake.rotator.rotateFully();
            else if(robot.outtake.getPower() < 0 && robot.outtake.getState() == Motor.State.DOWN && !robot.outtake.rotator.getState().equals(OuttakeRotator.RETRACTED))
                robot.outtake.rotator.retractFully();
            // slow down when nearing bottom
            if(robot.outtake.getPosition() - robot.outtake.getDownPosition() < 200 && robot.outtake.getPower() < -ARM_SLOW_SPEED)
                robot.outtake.setPower(-ARM_SLOW_SPEED);

            // rotate outtake
            if(gamepad.dpad_right && gamepad.start)
                robot.outtake.rotator.EXTENDED_PSN = robot.outtake.rotator.getPosition();
            else if(gamepad.dpad_left && gamepad.start)
                robot.outtake.rotator.RETRACTED_PSN = robot.outtake.rotator.getPosition();
            else if(gamepad.dpad_right && gamepad.back)
                robot.outtake.rotator.rotateIncrementally();
            else if(gamepad.dpad_left && gamepad.back)
                robot.outtake.rotator.retractIncrementally();
            else if(gamepad.dpad_right) {
                // make sure both claws are down first
                robot.claw1.down();
                robot.claw2.down();
                robot.outtake.rotator.rotateFully();
            } else if(gamepad.dpad_left)
                robot.outtake.rotator.retractFully();

            // hang
//            if(gamepad.start && gamepad.right_stick_y < 0)
//                robot.hangSubsystem.setUpPosition();
//            else if(gamepad.start && gamepad.right_stick_y > 0)
//                robot.hangSubsystem.setDownPosition();
//            else if(gamepad2.right_stick_y != 0 && gamepad2.back)
//                robot.hangSubsystem.runUsingEncoder();
//            else if((gamepad2.right_stick_y != 0 && !gamepad2.back && !gamepad2.start) || (gamepad1.back && gamepad1.right_stick_y != 0))
//                robot.hangSubsystem.runWithoutEncoder();

//            if(robot.hangSubsystem.isUsingEncoder()) {
//                if(gamepad2.right_stick_y < 0) {
//                    robot.hangSubsystem.goToUpPosition();
//                    hangTimer.reset();
//                } else if(gamepad2.right_stick_y > 0) {
//                    robot.hangSubsystem.goToDownPosition();
//                    hangTimer.reset();
//                } else if (robot.hangSubsystem.isIdle() || hangTimer.seconds() > 6)
//                    robot.hangSubsystem.stop();
            if((!singleDriverMode || gamepad1.back) && gamepad.right_stick_x < .3) {
                robot.hangSubsystem.setPowers(-gamepad.right_stick_y);
//                if(Math.abs(gamepad.right_stick_y) == 1)
//                    robot.hangSubsystem.setPowers(.8 * -gamepad.right_stick_y);
//                else if(Math.abs(gamepad.right_stick_y) > .4)
//                    robot.hangSubsystem.setPowers(.5 * gamepad.right_stick_y < 0 ? 1 : -1);
//                else if(Math.abs(gamepad.right_stick_y) > 0)
//                    robot.hangSubsystem.setPowers(.2 * gamepad.right_stick_y < 0 ? 1 : -1);
//                else if(!Arrays.equals(robot.hangSubsystem.getPowers(), new double[]{0, 0}))
//                    robot.hangSubsystem.setPowers(0);
                if(gamepad.right_stick_y < -.9 && robot.hangSubsystem.getRotatorState().equals(HangSubsystem.DOWN))
                    robot.hangSubsystem.rotateUp();
            }

            // rotate hang
            double threshold = .8;
            if(gamepad2.back && gamepad2.right_stick_x > threshold)
                robot.hangSubsystem.rotateUp(.01);
            else if(gamepad2.back && gamepad2.right_stick_x < -threshold)
                robot.hangSubsystem.rotateDown(.01);
            else if(gamepad2.right_stick_x > threshold || (singleDriverMode && gamepad1.back && gamepad1.right_stick_x > threshold))
                robot.hangSubsystem.rotateUp();
            else if(gamepad2.right_stick_x < -threshold || (singleDriverMode && gamepad1.back && gamepad1.right_stick_x < -threshold))
                robot.hangSubsystem.rotateDown();

            // launcher
            if((gamepad1.a && !singleDriverMode && !gamepad1.start && !gamepad1.back) || (gamepad.right_bumper && !gamepad.start && !gamepad.back)) {
                robot.launcher.launch();
                robot.launcher.updateLauncherTelemetry();
            } else if((gamepad1.b && !singleDriverMode && !gamepad1.start && !gamepad1.back) || (gamepad.left_bumper && !gamepad.start && !gamepad.back)) {
                robot.launcher.reset();
                robot.launcher.updateLauncherTelemetry();
            }
            if((gamepad.right_bumper && gamepad.back) || (!singleDriverMode && gamepad1.y)) {
                robot.launcher.rotateUp();
                robot.launcher.updateRotatorTelemetry();
            } else if((gamepad.left_bumper && gamepad.back) || (!singleDriverMode && gamepad1.x)) {
                robot.launcher.rotateDown();
                robot.launcher.updateRotatorTelemetry();
            }

            // auto claw
            if(gamepad.start && gamepad.y)
                robot.autoClaw.outAndIn();
            else if(gamepad.start && gamepad.x)
                robot.autoClaw.in();
            else if(gamepad.back && gamepad.y)
                robot.autoClaw.outIncrementally();
            else if(gamepad.back && gamepad.x)
                robot.autoClaw.inIncrementally();

            telemetry.update();
            loopTimer.reset();
        }
    }
    public static void setStartPose(Pose2d pose) { startPose = pose; }
}
