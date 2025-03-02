package org.firstinspires.ftc.teamcode.modules;


import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.modules.Drivetrain.RobotDirection.CCW;
import static java.lang.Double.min;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Drivetrain {
    private static final double WHEELS_DIAMETER = 96;
    private final static double DISTANCE_PER_ROTATION = WHEELS_DIAMETER * PI;
    private static final double PULSES_PER_ROTATION_312RPM = 537.7;
    private final static double TIC_PER_MM = PULSES_PER_ROTATION_312RPM / DISTANCE_PER_ROTATION;
    private final static double MM_PER_TICK = DISTANCE_PER_ROTATION / PULSES_PER_ROTATION_312RPM;
    private static final double ROTATIONAL_POWER = 1;
    private static final double MAX_DRIVE_DISTANCE = 3 * 24 * 2.45;
    private static final double MAX_TURN = 180;

    private static Telemetry telemetry;
    private static LinearOpMode opMode;
    public DcMotor leftFront, rightFront, leftBack, rightBack;

    private static Gyroscope gyroscope;

    public enum RobotDirection {
        FORWARD, BACK, LEFT, RIGHT, CW, CCW, LEFTFORWARD, LEFTBACK, RIGHTFORWARD, RIGHTBACK, STOP
    }

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
        telemetry = opMode.telemetry;
        HardwareMap hm = opMode.hardwareMap;
        gyroscope = new Gyroscope(opMode);
        leftFront = hm.get(DcMotor.class, "lf");
        rightFront = hm.get(DcMotor.class, "rf");
        leftBack = hm.get(DcMotor.class, "lb");
        rightBack = hm.get(DcMotor.class, "rb");
    }


    /**
     * Методы установки параметров мотора
     */

    public void setModes(RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    /**
     * Методы установки мощностей на моторы
     */
    public void setPower(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }


    public void setPower(double x, double y, double r) {
        setPower(x - y - r,
                x + y + r,
                x + y - r,
                x - y + r);
    }

    public void setPower(RobotDirection robotDirection, double power) {
        switch (robotDirection) {
            default:
            case STOP:
                setPower(0, 0, 0, 0);
            case FORWARD:
                setPower(-power, power, -power, power);
                break;
            case BACK:
                setPower(power, -power, power, -power);
                break;
            case LEFT:
                setPower(power, power, -power, -power);
                break;
            case RIGHT:
                setPower(-power, -power, power, power);
                break;
            case CW:
                setPower(-power, -power, -power, -power);
                break;
            case CCW:
                setPower(power, power, power, power);
                break;
            case LEFTFORWARD:
                setPower(0, power, -power, 0);
                break;
            case RIGHTFORWARD:
                setPower(-power, 0, 0, power);
                break;
            case LEFTBACK:
                setPower(power, 0, 0, -power);
                break;
            case RIGHTBACK:
                setPower(0, -power, power, 0);
                break;
        }
    }


    /**
     * Методы получения характеристик Колёсной Базы
     */
    private boolean isBusy() {
        return leftFront.getPower() != 0 || rightFront.getPower() != 0 || leftBack.getPower() != 0 || rightBack.getPower() != 0;
    }


    /**
     * Методы выведения характеристик в телеметрию
     */

    public void addTelemetry() {
        gyroscope.addTelemetry();
        telemetry.addLine("LF RF LB RB");
        telemetry.addLine(String.format("%s;%s;%s;", leftFront.getDirection(), leftFront.getMode(), leftFront.getZeroPowerBehavior()));
        telemetry.addLine(String.format("%s;%s", leftFront.getPower(), leftFront.getCurrentPosition()));
        // telemetry.addLine(String.format("%s;", motor.getVelocity()));
        telemetry.addLine(String.format("%s;%s;%s;", rightFront.getDirection(), rightFront.getMode(), rightFront.getZeroPowerBehavior()));
        telemetry.addLine(String.format("%s;%s", rightFront.getPower(), rightFront.getCurrentPosition()));
        // telemetry.addLine(String.format("%s;", motor.getVelocity()));
        telemetry.addLine(String.format("%s;%s;%s;", leftBack.getDirection(), leftBack.getMode(), leftBack.getZeroPowerBehavior()));
        telemetry.addLine(String.format("%s;%s", leftBack.getPower(), leftBack.getCurrentPosition()));
        // telemetry.addLine(String.format("%s;", motor.getVelocity()));
        telemetry.addLine(String.format("%s;%s;%s;", rightBack.getDirection(), rightBack.getMode(), rightBack.getZeroPowerBehavior()));
        telemetry.addLine(String.format("%s;%s", rightBack.getPower(), rightBack.getCurrentPosition()));
        // telemetry.addLine(String.format("%s;", motor.getVelocity()));
    }

    /**
     * Методы проезда
     */
    public void driveByEncoder(RobotDirection robotDirection, double power, double distance) {
        setModes(STOP_AND_RESET_ENCODER);
        setModes(RUN_USING_ENCODER);
        setPower(robotDirection, power);
        while (isBusy()) {
            telemetry.addLine("State:driveByEncoder");
            if (abs(leftFront.getCurrentPosition()) > distance * TIC_PER_MM) {
                leftFront.setPower(0);
            }
            if (abs(rightFront.getCurrentPosition()) > distance * TIC_PER_MM) {
                rightFront.setPower(0);
            }
            if (abs(leftBack.getCurrentPosition()) > distance * TIC_PER_MM) {
                leftBack.setPower(0);
            }
            if (abs(rightBack.getCurrentPosition()) > distance * TIC_PER_MM) {
                rightBack.setPower(0);
            }

            addTelemetry();
            telemetry.update();
        }
    }


    // трёхпозиционный релейный регулятор
    public void driveRReg(RobotDirection robotDirection, double power, double distance) {
        setModes(STOP_AND_RESET_ENCODER);
        setModes(RUN_USING_ENCODER);
        setPower(robotDirection, power);
        opMode.sleep(100);
        while (opMode.opModeIsActive() && isBusy() && abs(leftFront.getCurrentPosition() - distance * TIC_PER_MM) > 145) {
            telemetry.addLine("State:driveRReg");
            for (DcMotor motor : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
                motor.setPower(power * signum(distance * TIC_PER_MM * signum(motor.getCurrentPosition()) - motor.getCurrentPosition()));
                telemetry.addLine(String.format("%s;%s;", motor.getPower(), motor.getCurrentPosition()));
            }
            addTelemetry();
            telemetry.update();
        }
    }

    // пропорциональный регулятор
    public void drivePReg(RobotDirection robotDirection, double power, double distance) {
        setModes(STOP_AND_RESET_ENCODER);
        setModes(RUN_USING_ENCODER);
        setPower(robotDirection, power);
        opMode.sleep(100);
        while (isBusy()) {
            if (leftFront.getPower() != 0) {
                leftFront.setPower(power * (distance * signum(leftFront.getCurrentPosition()) - leftFront.getCurrentPosition() * MM_PER_TICK) / (MAX_DRIVE_DISTANCE));
            }
            telemetry.addLine("State:drivePReg");
            addTelemetry();
            telemetry.update();
            if (rightFront.getPower() != 0) {
                rightFront.setPower(power * (distance * signum(rightFront.getCurrentPosition()) - rightFront.getCurrentPosition() * MM_PER_TICK) / (MAX_DRIVE_DISTANCE));
            }
            telemetry.addLine("State:drivePReg");
            addTelemetry();
            telemetry.update();
            if (leftBack.getPower() != 0) {
                leftBack.setPower(power * (distance * signum(leftBack.getCurrentPosition()) - leftBack.getCurrentPosition() * MM_PER_TICK) / (MAX_DRIVE_DISTANCE));
            }
            telemetry.addLine("State:drivePReg");
            addTelemetry();
            telemetry.update();
            if (rightBack.getPower() != 0) {
                rightBack.setPower(power * (distance * signum(rightBack.getCurrentPosition()) - rightBack.getCurrentPosition() * MM_PER_TICK) / (MAX_DRIVE_DISTANCE));
            }
            telemetry.addLine("State:drivePReg");
            addTelemetry();
            telemetry.update();
        }
    }


    public void setTurnPRegByIMU(double angle) {
        setPower(CCW, ROTATIONAL_POWER * minAngle(angle, gyroscope.getHeading()) / (MAX_TURN));
        while (isBusy()) {
            telemetry.addLine("State:turnPRegByIMU");
            setPower(CCW, ROTATIONAL_POWER * minAngle(angle, gyroscope.getHeading()) / (MAX_TURN));

            addTelemetry();
            telemetry.update();
        }
    }

    public void turnPRegByIMU(double angle) {
        setTurnPRegByIMU(gyroscope.getHeading() + angle);
    }

    private double minAngle(double target, double pos) {
        double angle = target - pos;
        return min(angle, 360 - angle);
    }


}
