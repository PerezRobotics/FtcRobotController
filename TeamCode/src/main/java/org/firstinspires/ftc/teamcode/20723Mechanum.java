package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "MecanumDRIVE", group = "")
public class MecanumDRIVE extends LinearOpMode {

  private DcMotor mtr_rf;
  private DcMotor mtr_rr;
  private DcMotor mtr_lf;
  private DcMotor mtr_lr;
  private BNO055IMU imu_0;

  double left_x;
  double left_y;
  double right_x;
  double right_y;
  double bumper_speed;
  double ave_x;
  double power_rf;
  double power_rr;
  double power_lf;
  double power_lr;
  double scale;
  double tics_per_inch_forward;
  double tics_per_inch_sideways;
  double mtr_accel_min;
  double mtr_accel_tics;
  double mtr_decel_tics;
  double mtr_accel_degs;
  double mtr_decel_degs;
  Orientation angles;
  double heading;
  double heading_revs;
  float heading_raw_last;
  double start_heading;
  double mtr_decel_min;
  double auto_turn_rate;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double drive_mode;
    double last_drive_mode;
    double calibration_distance;
    BNO055IMU.Parameters imu_params;

    mtr_rf = hardwareMap.dcMotor.get("mtr_rf");
    mtr_rr = hardwareMap.dcMotor.get("mtr_rr");
    mtr_lf = hardwareMap.dcMotor.get("mtr_lf");
    mtr_lr = hardwareMap.dcMotor.get("mtr_lr");
    imu_0 = hardwareMap.get(BNO055IMU.class, "imu_0");

    // Reset and enable motor encoders. drive_mode = 0 is tank
    drive_mode = 0;
    last_drive_mode = 0;
    bumper_speed = 0.5;
    mtr_accel_min = 0.3;
    mtr_decel_min = 0.1;
    mtr_accel_tics = 600;
    mtr_decel_tics = 1200;
    mtr_accel_degs = 20;
    mtr_decel_degs = 30;
    auto_turn_rate = 0.15;
    calibration_distance = 24;
    tics_per_inch_forward = 84;
    tics_per_inch_sideways = 83;
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    mtr_rf.setDirection(DcMotorSimple.Direction.FORWARD);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    mtr_rr.setDirection(DcMotorSimple.Direction.FORWARD);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    mtr_lf.setDirection(DcMotorSimple.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    mtr_lr.setDirection(DcMotorSimple.Direction.REVERSE);
    mtr_rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mtr_rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mtr_lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mtr_lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    reset_drive_encoders();
    imu_params = new BNO055IMU.Parameters();
    imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    imu_params.calibrationDataFile = "BNO055IMUCalibration.json";
    waitForStart();
    imu_0.initialize(imu_params);
    while (true) {
      if (imu_0.isGyroCalibrated()) {
        break;
      }
    }
    heading_revs = 0;
    angles = imu_0.getAngularOrientation();
    heading_raw_last = angles.firstAngle;
    heading = -heading_raw_last;
    // This is the main run loop, we start in tank mode, but use the B button to switch to airplane, use X to exit
    while (opModeIsActive()) {
      // Test for finished, test for switch between tank and aircraft
      if (gamepad1.x) {
        break;
      } else if (gamepad1.b) {
        if (drive_mode == last_drive_mode) {
          drive_mode = (drive_mode + 1) % 5;
        }
      } else if (gamepad1.dpad_up) {
        // Move forward the calibration distance
        move(calibration_distance, 0);
      } else if (gamepad1.dpad_down) {
        // Move backwards the calibration distance
        move(-calibration_distance, 0);
      } else if (gamepad1.dpad_right) {
        if (gamepad1.left_bumper) {
          // Rotate 90 clockwise
          rotate(90);
        } else {
          // Move right the calibration distance
          move(calibration_distance, 90);
        }
      } else if (gamepad1.dpad_left) {
        if (gamepad1.left_bumper) {
          // Rotate 90 counter-clockwise
          rotate(-90);
        } else {
          // Move left the calibration distance
          move(calibration_distance, -90);
        }
      } else {
        last_drive_mode = drive_mode;
        // This is the real drive loop
        // Condition the X-Y stick inputs
        condition_sticks();
        // Set the motor speeds
        if (drive_mode == 0) {
          // Tank drive - motor speeds are stick Y for both left and right
          tank_drive();
        } else if (drive_mode == 1) {
          // right stick is forward/back and side-to-side with no rotation, left X is rotation.
          airplane_right();
        } else if (drive_mode == 2) {
          // left stick is forward/back and side-to-side with no rotation, right X is rotation.
          airplane_left();
        } else if (drive_mode == 3) {
          // A little like a car, right stick is forward-backward and turn
          auto_right();
        } else {
          // A little like a car, left stick is forward-backward and turn
          auto_left();
        }
      }
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void condition_sticks() {
    left_x = gamepad1.left_stick_x;
    left_y = -gamepad1.left_stick_y;
    right_x = gamepad1.right_stick_x;
    right_y = -gamepad1.right_stick_y;
    if (gamepad1.right_bumper) {
      right_x = right_x * bumper_speed;
      right_y = right_y * bumper_speed;
      left_x = left_x * bumper_speed;
      left_y = left_y * bumper_speed;
    }
    ave_x = (left_x + right_x) * 0.5;
  }

  /**
   * Describe this function...
   */
  private void tank_drive() {
    double max_left;
    double max_right;

    telemetry.addData("drive mode", "tank");
    scale = 1;
    max_left = Math.abs(left_y) + Math.abs(ave_x);
    max_right = Math.abs(right_y) + Math.abs(ave_x);
    if (max_left > 1) {
      if (max_right > max_left) {
        scale = 1 / max_right;
      } else {
        scale = 1 / max_left;
      }
    } else if (max_right > 1) {
      scale = 1 / max_right;
    }
    power_rf = scale * (right_y - ave_x);
    power_rr = scale * (right_y + ave_x);
    power_lf = scale * (left_y + ave_x);
    power_lr = scale * (left_y - ave_x);
    set_power();
  }

  /**
   * Describe this function...
   */
  private double power_accel_decel(double current_tics, double target_tics, double mtr_accel_min, double accel_tics, double decel_tics) {
    double mtr_tmp;
    double mtr_tmp_2;

    if (current_tics <= 0) {
      return mtr_accel_min;
    }
    if (current_tics >= target_tics) {
      return 0;
    }
    mtr_tmp = 1;
    if (current_tics < accel_tics) {
      mtr_tmp = mtr_accel_min + (1 - mtr_accel_min) * (current_tics / accel_tics);
    }
    if (current_tics > target_tics - decel_tics) {
      mtr_tmp_2 = mtr_decel_min + (1 - mtr_decel_min) * ((target_tics - current_tics) / decel_tics);
      if (mtr_tmp_2 < mtr_tmp) {
        mtr_tmp = mtr_tmp_2;
      }
    }
    return mtr_tmp;
  }

  /**
   * Describe this function...
   */
  private double forward_tics() {
    return mtr_rf.getCurrentPosition() + mtr_lf.getCurrentPosition() + mtr_rr.getCurrentPosition() + mtr_lr.getCurrentPosition();
  }

  /**
   * Describe this function...
   */
  private double sideways_tics() {
    return (mtr_rr.getCurrentPosition() + mtr_lf.getCurrentPosition()) - (mtr_rf.getCurrentPosition() + mtr_lr.getCurrentPosition());
  }

  /**
   * Describe this function...
   */
  private void airplane_right() {
    // The deal here is that right stick is forward and sideways, left X is turn.
    telemetry.addData("drive mode", "airplane right");
    set_speeds(right_y, right_x, left_x);
  }

  /**
   * Describe this function...
   */
  private void move(double inches, double degrees) {
    double current_tics;
    double target_tics;
    double cos;
    double sin;
    double forward_max_speed;
    double forward_inches;
    double forward_direction_mult;
    double sideways_max_speed;
    double sideways_inches;
    double sideways_direction_mult;
    double speed_mult;

    reset_drive_encoders();
    sin = Math.sin(degrees / 180 * Math.PI);
    cos = Math.cos(degrees / 180 * Math.PI);
    forward_max_speed = Math.abs(cos);
    forward_inches = cos * inches;
    if (forward_inches > 0) {
      forward_direction_mult = 1;
    } else {
      forward_direction_mult = -1;
    }
    sideways_max_speed = Math.abs(sin);
    sideways_inches = sin * inches;
    if (sideways_inches > 0) {
      sideways_direction_mult = 1;
    } else {
      sideways_direction_mult = -1;
    }
    if (forward_max_speed >= sideways_max_speed) {
      target_tics = tics_per_inch_forward * forward_inches * forward_direction_mult;
    } else {
      target_tics = tics_per_inch_sideways * sideways_inches * sideways_direction_mult;
    }
    while (true) {
      if (forward_max_speed >= sideways_max_speed) {
        current_tics = forward_tics() * forward_direction_mult;
      } else {
        current_tics = sideways_tics() * sideways_direction_mult;
      }
      if (current_tics >= target_tics) {
        break;
      }
      speed_mult = power_accel_decel(current_tics, target_tics, mtr_accel_min, mtr_accel_tics, mtr_decel_tics);
      set_speeds(forward_max_speed * speed_mult * forward_direction_mult, sideways_max_speed * speed_mult * sideways_direction_mult, 0);
    }
    set_speeds(0, 0, 0);
  }

  /**
   * Describe this function...
   */
  private void set_power() {
    float heading_raw;

    mtr_rf.setPower(power_rf);
    mtr_rr.setPower(power_rr);
    mtr_lf.setPower(power_lf);
    mtr_lr.setPower(power_lr);
    angles = imu_0.getAngularOrientation();
    heading_raw = angles.firstAngle;
    if (heading_raw_last < -150 && heading_raw > 0) {
      heading_revs += -1;
    } else if (heading_raw_last > 150 && heading_raw < 0) {
      heading_revs += 1;
    }
    heading = -(heading_revs * 360 + heading_raw);
    heading_raw_last = heading_raw;
    telemetry.addData("revs", heading_revs);
    telemetry.addData("heading", heading);
  }

  /**
   * Describe this function...
   */
  private void airplane_left() {
    // The deal here is that left stick is forward and sideways, right X is turn.
    telemetry.addData("drive mode", "airplane left");
    set_speeds(left_y, left_x, right_x);
  }

  /**
   * Describe this function...
   */
  private void auto_right() {
    // A little like a car, right stick is forward-backward and turn
    telemetry.addData("drive mode", "auto right");
    set_speeds(right_y, left_x, right_x + right_x * right_y * (auto_turn_rate - 1));
  }

  /**
   * Describe this function...
   */
  private void auto_left() {
    // A little like a car, left stick is forward-backward and turn
    telemetry.addData("drive mode", "auto left");
    set_speeds(left_y, right_x, left_x + left_x * left_y * (auto_turn_rate - 1));
  }

  /**
   * Describe this function...
   */
  private void set_speeds(double forward, double sideways, double rotation) {
    double max;

    scale = 1;
    max = Math.abs(forward) + Math.abs(sideways) + Math.abs(rotation);
    if (max > 1) {
      scale = 1 / max;
    }
    power_rf = scale * (forward - (sideways + rotation));
    power_rr = scale * (forward + (sideways - rotation));
    power_lf = scale * (forward + sideways + rotation);
    power_lr = scale * (forward - (sideways - rotation));
    set_power();
  }

  /**
   * Describe this function...
   */
  private void rotate(double degrees) {
    start_heading = heading;
    // Rotate as specified
    turn(degrees);
    // Test the heading and correct for error
    turn(degrees - (heading - start_heading));
  }

  /**
   * Describe this function...
   */
  private void reset_drive_encoders() {
    mtr_rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mtr_rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mtr_lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mtr_lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mtr_rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mtr_rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mtr_lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mtr_lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  /**
   * Describe this function...
   */
  private void turn(double degrees) {
    double direction_mult;

    reset_drive_encoders();
    if (degrees > 0) {
      direction_mult = 1;
    } else {
      direction_mult = -1;
    }
    start_heading = heading;
    while (direction_mult * (heading - start_heading) < degrees * direction_mult) {
      set_speeds(0, 0, power_accel_decel(direction_mult * (heading - start_heading), degrees * direction_mult, mtr_accel_min, mtr_accel_degs, mtr_decel_degs) * direction_mult);
    }
    set_speeds(0, 0, 0);
  }
