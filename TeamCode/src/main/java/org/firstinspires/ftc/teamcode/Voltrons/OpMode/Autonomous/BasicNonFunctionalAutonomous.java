package org.firstinspires.ftc.teamcode.Voltrons.OpMode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Voltrons.Vision.RingPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Config
@Autonomous(name="NonoAutonomous", group="auto")
public class BasicNonFunctionalAutonomous extends LinearOpMode {

    OpenCvCamera phoneCam;
    RingPipeline visionPipeline;
    BNO055IMU imu;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    DcMotor left_front;
    DcMotor right_front;
    DcMotor left_back;
    DcMotor right_back;

    DcMotor wooble_arm;
    Servo wooble_hand;

    DcMotor intake;
    CRServo belt_down;
    CRServo belt_up;
    DcMotor left_launcher;
    DcMotor right_launcher;

    public static int A1E = 8000;
    /*
    public static int A2I = 3000;
    public static int A3D = 3000;


    public static int A4A = 5000;
    public static int A5I = 2000;
    public static int A6D = 5000;
    public static int A7E = 5000;
    public static int A8I = 5000;
    public static int A9A = 5000;
    */

    public static int B1E = 8000;
    /*
    public static int B2I = 3000;

    public static int B3D = 3000;
    public static int B4A = 5000;
    public static int B5I = 2000;
    public static int B6D = 5000;
    public static int B7E = 5000;
    public static int B8I = 5000;
    public static int B9A = 5000;
    */

    public static int C1E = 8000;

    /*
    public static int C2I = 3000;
    public static int C3D = 3000;

    public static int C4A = 5000;
    public static int C5I = 2000;
    public static int C6D = 5000;
    public static int C7E = 5000;
    public static int C8I = 5000;
    public static int C9A = 5000;
    */

    public void runOpMode()
    {
        // Init Camera and set pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        visionPipeline = new RingPipeline();
        phoneCam.setPipeline(visionPipeline);
        phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        FtcDashboard.getInstance().startCameraStream(phoneCam,60);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Motor Stuff


        left_front = hardwareMap.dcMotor.get("lf");
        right_front = hardwareMap.dcMotor.get("rf");
        left_back = hardwareMap.dcMotor.get("lb");
        right_back = hardwareMap.dcMotor.get("rb");

        wooble_arm = hardwareMap.dcMotor.get("wa");
        wooble_hand = hardwareMap.servo.get("wh");

        intake = hardwareMap.dcMotor.get("in");
        belt_down = hardwareMap.crservo.get("bd");
        belt_up = hardwareMap.crservo.get("bu");

        // 1
        right_launcher = hardwareMap.dcMotor.get("rl");
        // 0
        left_launcher = hardwareMap.dcMotor.get("ll");

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);

        wooble_hand.setPosition(0);

        double k_p = 0.002;
        waitForStart();
        if (opModeIsActive())
        {
            packet.put("Ring 1", visionPipeline.ring1);
            packet.put("Ring 4", visionPipeline.ring4);
            dashboard.sendTelemetryPacket(packet);


            // Uno es cerrado, 0 es abierto
            double position_goal = 400;

           forward(A1E,0.5);
        }
    }

    //TODO: Agregar PID tanto de posicion como de correcion de angulo
    void forward(int ticks, double power)
    {
        int new_distance = right_back.getCurrentPosition() + ticks;

        left_back.setPower(power);
        right_back.setPower(power);
        left_front.setPower(power);
        right_front.setPower(power);

        while (right_back.getCurrentPosition() < new_distance) {
            packet.put("Encoder Izquierdo", left_back.getCurrentPosition());
            packet.put("Encoder Derecho", right_back.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }

        left_back.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        right_front.setPower(0);
    }

    void backwards(int ticks, double power)
    {
        int new_distance = right_back.getCurrentPosition() - ticks;

        left_back.setPower(-power);
        right_back.setPower(-power);
        left_front.setPower(-power);
        right_front.setPower(-power);

        while (right_back.getCurrentPosition() > new_distance) {
            packet.put("Encoder Izquierdo", left_back.getCurrentPosition());
            packet.put("Encoder Derecho", right_back.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }

        left_back.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        right_front.setPower(0);
    }

    void left(int ticks, double power)
    {
        int new_distance = right_back.getCurrentPosition() - ticks;

        left_back.setPower(power);
        right_back.setPower(-power); // Encoder que me importa. Va a ir hacia atras
        left_front.setPower(-power);
        right_front.setPower(power);

        while (right_back.getCurrentPosition() > new_distance) {
            packet.put("Encoder Izquierdo", left_back.getCurrentPosition());
            packet.put("Encoder Derecho", right_back.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }

        left_back.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        right_front.setPower(0);
    }

    void right(int ticks, double power)
    {
        int new_distance = right_back.getCurrentPosition() + ticks;

        left_back.setPower(-power);
        right_back.setPower(power);
        left_front.setPower(power);
        right_front.setPower(-power);

        while (right_back.getCurrentPosition() < new_distance) {
            packet.put("Encoder Izquierdo", left_back.getCurrentPosition());
            packet.put("Encoder Derecho", right_back.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }

        left_back.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        right_front.setPower(0);
    }

    double getHeading()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    void turn_right(double power)
    {
        left_back.setPower(power);
        right_back.setPower(-power);
        left_front.setPower(power);
        right_front.setPower(-power);
        while(true)
        {
            if (getHeading() > 0)break;
        }
        left_back.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        right_front.setPower(0);
    }
}
