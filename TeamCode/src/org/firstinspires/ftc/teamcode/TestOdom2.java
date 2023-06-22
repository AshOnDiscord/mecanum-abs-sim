package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp op mode to test odometry with three "dead-wheel" encoders. This op mode will work with
 * either the MecBot or the XDriveBot robot configuration.
 */
@TeleOp(name = "TestOdom2", group = "OdomBot")
public class TestOdom2 extends LinearOpMode {

    EncBot bot = new EncBot();
    double[] pose;
    Pos dir = new Pos(0, 0);
    double y = 0;
    double x = 0;

    public void runOpMode() {
        bot.init(hardwareMap);
        bot.resetOdometry(0, 0, 90);
//        bot.resetOdometry(0, 0, 45);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("time", this.time);
            telemetry.update();
        }

//        while (opModeIsActive()) {
//            double px = gamepad1.left_stick_x;
//            double py = -gamepad1.left_stick_y;
//            double pa = gamepad1.left_trigger - gamepad1.right_trigger;
//            bot.setDrivePower(px, py, pa);
//            updateTelemetry();
//        }
        for (float i = 0; i <= 360; i+= 22.5) {
            rotate(90 - i);
            testPos();
        }
    }

    public void testPos() {
        sleep(500);
        absoluteMove(new Pos(25, 25));
        sleep(500);
        absoluteMove(new Pos(25/2f, -25));
        sleep(500);
        absoluteMove(new Pos(-25/2f, 25/2f));
        sleep(500);
        absoluteMove(new Pos(-25, -25/2f));
        sleep(500);
        absoluteMove(new Pos(0, 0));
        sleep(500);
    }

    public void absoluteMove(Pos target) {
        pose = bot.updateOdometry();
        double[] start = {pose[0], pose[1], pose[2]};
//        double[] start = {0, 0, 45};

//        double angleToTarget2 = Math.toDegrees(Math.atan((target.x - start[0]) / (target.y - start[1])));
        double angleToTarget2 = Math.toDegrees(Math.atan2(target.y-start[1], target.x-start[0]));
        double angleToAngle2 = angleToTarget2 - start[2];
        double dist2 = Math.sqrt(Math.pow(target.x - start[0], 2) + Math.pow(target.y - start[1], 2));
        double x2 = Math.sin(Math.toRadians(angleToAngle2));
        double y2 = Math.cos(Math.toRadians(angleToAngle2));
        System.out.println(angleToTarget2 + " " + angleToAngle2 + " | " + x2 + " " + y2 + " | " + dist2 + " | " + (dist2 * x2) + " " + (dist2 * y2));
//        return;
        double tolerance = 2;
        while (opModeIsActive()) {
            if (Math.abs(target.x - pose[0]) > tolerance || Math.abs(target.y - pose[1]) > tolerance) {
//                if (Math.abs(target.x - pose[0]) > tolerance) {
//                    System.out.println("x: " + pose[0] + " " + target.x + " " + Math.abs(target.x - pose[0]));
//                }
//                if (Math.abs(target.y - pose[1]) > tolerance) {
//                    System.out.println("y: " + pose[1] + " " + target.y + " " + Math.abs(target.y - pose[1]));
//                }
                pose = bot.updateOdometry();
                double angleToTarget = Math.toDegrees(Math.atan2(target.y-start[1], target.x-start[0]));
                double angleToAngle = angleToTarget - start[2];
                double dist = Math.sqrt(Math.pow(target.x - start[0], 2) + Math.pow(target.y - start[1], 2));
                double x = -Math.toDegrees(Math.sin(Math.toRadians(angleToAngle)));
                double y = Math.toDegrees(Math.cos(Math.toRadians(angleToAngle)));
                double rx = 0;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

//                System.out.println("\u001B[33m" + x + " " + y + " | " + dir.x + " " + dir.y + "\u001B[0m");
//                System.out.println("\u001B[32m" + xBase + " " + yBase + " | " + xDist + " " + yDist + " | " + deno + "\u001B[0m");

                // // Denominator is the largest motor power (absolute value) or 1
                // // This ensures all the powers maintain the same ratio, but only when
                // // at least one is out of the range [-1, 1]
                double frontLeftPower = (y + x + rx);
                double backLeftPower = (y - x + rx);
                double frontRightPower = (y - x - rx);
                double backRightPower = (y + x - rx);
                frontLeftPower = frontLeftPower / denominator;
                backLeftPower = backLeftPower / denominator;
                frontRightPower = frontRightPower / denominator;
                backRightPower = backRightPower / denominator;

                bot.motors[1].setPower(frontLeftPower);
                bot.motors[0].setPower(backLeftPower);
                bot.motors[2].setPower(frontRightPower);
                bot.motors[3].setPower(backRightPower);
                updateTelemetry();

//                sleep(50);
            } else {
                break;
            }
        }
        bot.motors[1].setPower(0);
        bot.motors[0].setPower(0);
        bot.motors[2].setPower(0);
        bot.motors[3].setPower(0);
    }

    public void updateTelemetry() {
        pose = bot.updateOdometry();
        telemetry.addData("time", this.time);
        telemetry.addData("POSE", "x = %.1f  y = %.1f  h = %.1f", pose[0], pose[1], pose[2]);
        telemetry.addData("Back Left", "T = %d  V = %.0f", bot.motors[0].getCurrentPosition(), bot.motors[0].getVelocity());
        telemetry.addData("Front Left", "T = %d  V = %.0f", bot.motors[1].getCurrentPosition(), bot.motors[1].getVelocity());
        telemetry.addData("Front Right", "T = %d  V = %.0f", bot.motors[2].getCurrentPosition(), bot.motors[2].getVelocity());
        telemetry.addData("Back Right", "T = %d  V = %.0f", bot.motors[3].getCurrentPosition(), bot.motors[3].getVelocity());
        telemetry.addData("Right Enc", "T = %d  V = %.0f", bot.encoders[0].getCurrentPosition(), bot.encoders[0].getVelocity());
        telemetry.addData("Left Enc", "T = %d  V = %.0f", bot.encoders[1].getCurrentPosition(), bot.encoders[1].getVelocity());
        telemetry.addData("X Enc", "T = %d  V = %.0f", bot.encoders[2].getCurrentPosition(), bot.encoders[2].getVelocity());
        telemetry.addData("dir ", "x = %.1f  y = %.1f", dir.x, dir.y);
        telemetry.addData("move ", "x = %.1f  y = %.1f", x, y);
        telemetry.addData("IMU POSE", bot.imu.getAngularOrientation().firstAngle);
        telemetry.update();
    }

    public void rotate(float target) {
//        float rotation = bot.imu.getAngularOrientation().firstAngle;
        pose = bot.updateOdometry();
        float rotation = (float) pose[2];
        float tolerance = 0.5f;
        float delay = 12.5f;
        float attempts = 160;
        // snappier
        // float kp = 0.02;
        // float ki = 0.00000;
        // float kd = 0.05;
        // smoother but bit slow end
        float kp = 0.05f;
        float ki = 0.00000f;
        float kd = 0.2f;
        float i = 0;
        float lastError = 0;
        while (opModeIsActive()) {
            if (Math.abs(target - rotation) > tolerance) {
//                rotation = bot.imu.getAngularOrientation().firstAngle;
                pose = bot.updateOdometry();
                rotation = (float) pose[2];
                float error = -((target - rotation + 540) % 360) - 180;
                if (Math.abs(error) > 180) {
                    if (error > 0) {
                        error -= 360;
                    } else {
                        error += 360;
                    }
                }
                i += (error * delay);
                float d = (error - lastError);
                float power = kp * error + ki * i + kd * d;
                float n = power;
                if (power > 0) {
                    power = (float) Math.min(1, Math.max(0.01, power));
                } else {
                    power = (float) Math.max(-1, Math.min(-0.01, power));
                }
                telemetry.addData("Capped Power", n);
                telemetry.addData("Power", power);
                telemetry.addData("p", kp * error);
                telemetry.addData("i", ki * i);
                telemetry.addData("d", kd * d);
                telemetry.addData("angle", pose[2]);
//                telemetry.addData("first", -bot.imu.getAngularOrientation().firstAngle);
//                telemetry.addData("second", -bot.imu.getAngularOrientation().secondAngle);
//                telemetry.addData("third", -bot.imu.getAngularOrientation().thirdAngle);
                telemetry.update();
                bot.motors[1].setPower(power);
                bot.motors[0].setPower(power);
                bot.motors[2].setPower(-power);
                bot.motors[3].setPower(-power);
                lastError = error;
                sleep((long) delay);
                attempts--;
                if (attempts <= 0) {
                    System.out.println(power);
                    System.out.println(error);
                    break;
                }
//                for (int k = 0; k < 10 && opModeIsActive(); k++) {
//                    if (Math.abs(target - rotation) < tolerance) {
//                        sleep(10);
//                        rotation = -bot.imu.getAngularOrientation().firstAngle;
//                    } else {
//                        break;
//                    }
//                }
            } else {
//                stop();
                bot.motors[0].setPower(0);
                bot.motors[1].setPower(0);
                bot.motors[2].setPower(0);
                bot.motors[3].setPower(0);
                telemetry.addData("done", "done");
                telemetry.update();
                return;
            }
        }
    }

    public Pos ab(Pos a, Pose b) {
        double x1 = a.x;
        double y1 = a.y;
        double x2 = b.x;
        double y2 = b.y;
//        double alp2 = b.angle;
//        alp2 = (Math.atan((x2 - x1) / (y2 - y1)) * 180) / Math.PI - 12;
//        double alp2 = (Math.atan((x2 - x1) / (y2 - y1)) * 180) / Math.PI - 90 - b.angle;
        double alp2 = (Math.atan((x2 - x1) / (y2 - y1)) * 180) / Math.PI - b.angle;
//        double alp2 = 45;
        double alp1 = 90 - alp2;
//        System.out.println(alp2 + " " + alp1);
        alp1 *= Math.PI / 180;
        alp2 *= Math.PI / 180;
        double u = x2 - x1;
        double v = y2 - y1;
        double a3 = Math.sqrt(u * u + v * v);
        double alp3 = Math.PI - alp1 - alp2;
        double a2 = (a3 * Math.sin(alp2)) / Math.sin(alp3);
        double RHS1 = x1 * u + y1 * v + a2 * a3 * Math.cos(alp1);
        double RHS2 = y2 * u - x2 * v - a2 * a3 * Math.sin(alp1);
        double x3 = (1 / (a3 * a3)) * (u * RHS1 - v * RHS2);
        double y3 = (1 / (a3 * a3)) * (v * RHS1 + u * RHS2);
        // 1/900 = 0.001
        // 0*0+1
//        System.out.println(a3 + " | " + u + " | " + v + " | " + RHS1 + " | " + RHS2 + " | " + x3 + " | " + y3);
//        System.out.println(x3 + " " + y3);

//        return new Pos(x3, -y3);
        return new Pos(x3, y3);
    }

    public static class Pose {
        public double x;
        public double y;
        public double angle;

        public Pose(double x, double y, double angle) {
            this.x = x;
            this.y = y;
            this.angle = angle;
        }
    }

    public static class Pos {
        public double x;
        public double y;

        public Pos(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
