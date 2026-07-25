package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class CanAndGyroLocalizer implements Localizer {

    // ---- odometry hardware ----
    private final DcMotorEx parMotor, perpMotor;
    public double parYTicks = 3247.741675;  // from your RoadRunner TwoDeadWheelLocalizer.Params
    public double perpXTicks = 0;
    public double inPerTick = 0.001966233946; // from RoadRunner MecanumDrive.Params.inPerTick

    private int lastParPos, lastPerpPos;
    private Pose currentPose;
    private Pose currentVelocity = new Pose(0, 0, 0);
    private boolean initialized = false;

    // ---- gyro hardware (folded in from your standalone CanAndGyro sensor class) ----
    private final AnalogInput gyroSensor;
    private double zeroPoint;
    private double currentHeading = 0;   // wrapped -pi..pi
    private double totalHeading = 0;     // unwrapped, accumulates every update
    private double lastGyroHeading = 0;
    private double currentAngularVelocity = 0;
    private long lastNanoTime = -1;

    private static final int FILTER_SIZE = 10;
    private final double[] velocityHistory = new double[FILTER_SIZE];
    private int filterIndex = 0;
    private boolean filterFilled = false;

    public CanAndGyroLocalizer(HardwareMap hardwareMap, Pose startPose) {
        parMotor = hardwareMap.get(DcMotorEx.class, "EH0");
        perpMotor = hardwareMap.get(DcMotorEx.class, "CH0");
        perpMotor.setDirection(DcMotorEx.Direction.REVERSE);

        gyroSensor = hardwareMap.get(AnalogInput.class, "CHA0");
        calibrateGyro();

        currentPose = startPose;
    }

    private void calibrateGyro() {
        zeroPoint = gyroSensor.getVoltage();
        lastNanoTime = -1;
        currentHeading = 0;
        totalHeading = 0;
        lastGyroHeading = 0;
        currentAngularVelocity = 0;
        Arrays.fill(velocityHistory, 0);
        filterIndex = 0;
        filterFilled = false;
    }

    private void updateGyro() {
        long now = System.nanoTime();

        currentHeading = Math.IEEEremainder(
                2 * Math.PI * ((gyroSensor.getVoltage() - zeroPoint) / 3.3),
                2 * Math.PI
        );

        if (lastNanoTime < 0) {
            lastGyroHeading = currentHeading;
            lastNanoTime = now;
            currentAngularVelocity = 0.0;
            return;
        }

        double dt = (now - lastNanoTime) / 1e9;
        if (dt > 0.00001) {
            double delta = currentHeading - lastGyroHeading;
            if (delta > Math.PI) delta -= 2 * Math.PI;
            if (delta < -Math.PI) delta += 2 * Math.PI;

            totalHeading += delta; // accumulate unwrapped

            currentAngularVelocity = delta / dt;
            velocityHistory[filterIndex] = currentAngularVelocity;
            filterIndex = (filterIndex + 1) % FILTER_SIZE;
            if (filterIndex == 0) filterFilled = true;

            double sum = 0;
            int count = filterFilled ? FILTER_SIZE : filterIndex;
            for (int i = 0; i < count; i++) sum += velocityHistory[i];
            currentAngularVelocity = sum / Math.max(count, 1);

            lastGyroHeading = currentHeading;
            lastNanoTime = now;
        }
    }

    @Override
    public Pose getPose() {
        return currentPose;
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        Vector v = new Vector();
        v.setOrthogonalComponents(currentVelocity.getX(), currentVelocity.getY());
        return v;
    }

    @Override
    public void setStartPose(Pose setStart) {
        currentPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        currentPose = setPose;
    }

    @Override
    public void update() {
        updateGyro();

        int parPos = parMotor.getCurrentPosition();
        int perpPos = perpMotor.getCurrentPosition();
        double heading = currentHeading;

        if (!initialized) {
            initialized = true;
            lastParPos = parPos;
            lastPerpPos = perpPos;
            return;
        }

        double parDelta = parPos - lastParPos;
        double perpDelta = perpPos - lastPerpPos;
        double headingDelta = currentAngularVelocity; // rad/s, used below for dt-scaled velocity only

        double forward = (parDelta - parYTicks * (totalHeading - (totalHeading))) * inPerTick; // heading delta handled via par/perp geometry below
        // Simplified straight port of your TwoDeadWheelLocalizer math using raw deltas:
        double rawHeadingDelta = 0; // recompute properly:
        // recompute heading delta directly from encoder-independent gyro accumulation this loop:
        // (kept simple/explicit rather than reusing headingDelta var name ambiguously)
        rawHeadingDelta = currentHeading - lastGyroHeadingForOdom;
        if (rawHeadingDelta > Math.PI) rawHeadingDelta -= 2 * Math.PI;
        if (rawHeadingDelta < -Math.PI) rawHeadingDelta += 2 * Math.PI;

        forward = (parDelta - parYTicks * rawHeadingDelta) * inPerTick;
        double strafe = (perpDelta - perpXTicks * rawHeadingDelta) * inPerTick;

        double cos = Math.cos(currentPose.getHeading());
        double sin = Math.sin(currentPose.getHeading());
        double dx = forward * cos - strafe * sin;
        double dy = forward * sin + strafe * cos;

        currentVelocity = new Pose(dx, dy, rawHeadingDelta);
        currentPose = new Pose(currentPose.getX() + dx, currentPose.getY() + dy, heading);

        lastParPos = parPos;
        lastPerpPos = perpPos;
        lastGyroHeadingForOdom = currentHeading;
    }
    private double lastGyroHeadingForOdom = 0;

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return inPerTick;
    }

    @Override
    public double getLateralMultiplier() {
        return inPerTick;
    }

    @Override
    public double getTurningMultiplier() {
        return 1.0;
    }

    @Override
    public void resetIMU() {
        calibrateGyro();
    }

    @Override
    public double getIMUHeading() {
        return currentHeading;
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPose.getX())
                || Double.isNaN(currentPose.getY())
                || Double.isNaN(currentPose.getHeading());
    }
}