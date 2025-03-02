package org.firstinspires.ftc.teamcode.modules;


import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Guides {
    private static final double COIL_DIAMETR = 40;
    private static final double PULSES_PER_ROTATION_312RPM = 537.7;
    static double STAY_POWER;
    static double MAX_LENGTH;
    public double LIFT_POWER;
    private Telemetry telemetry;
    private DcMotor motor;
    private double MM_PER_TICS;
    private String motorName;

    public Guides(LinearOpMode opMode, String motorName, DcMotor.Direction direction, double LIFT_POWER, double STAY_POWER, double MAX_LENGTH) {
        this.telemetry = opMode.telemetry;
        motor = opMode.hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MM_PER_TICS = (COIL_DIAMETR * PI) / PULSES_PER_ROTATION_312RPM;
        this.motorName = motorName;
        this.LIFT_POWER = LIFT_POWER;
        this.STAY_POWER = STAY_POWER;
        this.MAX_LENGTH = MAX_LENGTH;
    }

    public void addTelemetry() {
        telemetry.addData(motorName + ":Power", motor.getPower());
        telemetry.addData(motorName + ":Position", motor.getCurrentPosition());
        telemetry.addData(motorName + ":Length", getLength());
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public double getLength() {
        return getPosition() * MM_PER_TICS;
    }


    public void setByRREG(double target, double power, double precision) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (abs(target - getPosition()) > precision) {
            if (target - getPosition() > 0) {
                setPower(power);
            } else {
                setPower(-power);
            }
        }
        setPower(0);
    }

    public void setByRREGV2(double target, double power, double precision) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (abs(target - getPosition()) > precision) {
            setPower(power * signum(target - getPosition()));
        }
        setPower(0);
    }

    public void rRegInMM(double length, double power) {
        while (getLength() != length) {
            motor.setPower(power * signum(length - getLength()));
        }
        motor.setPower(STAY_POWER);
    }

    public void pRegSet(double length) {
        while (getLength() != length) {
            motor.setPower(LIFT_POWER * (length - getLength()) / (MAX_LENGTH / 2));
        }
        motor.setPower(STAY_POWER);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
