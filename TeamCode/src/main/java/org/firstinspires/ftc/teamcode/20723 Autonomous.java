package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates using a webcam to locate and drive towards ANY Vuforia target.
 * The code assumes a basic two-wheel Robot Configuration with motors named left_drive and right_drive.
 * The motor directions must be set so a positive drive goes forward and a positive turn rotates to the right.
 *
 * Under manual control, the left stick will move forward/back, and the right stick will turn left/right.
 * This is called POV Joystick mode, different than Tank Drive (where each joystick controls a wheel).
 * Manually drive the robot until it displays Target data on the Driver Station.
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN and TURN_GAIN constants.
 *
 * For more Vuforia details, or to adapt this OpMode for a phone camera, view the
 *  ConceptVuforiaFieldNavigation and ConceptVuforiaFieldNavigationWebcam samples.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name="Drive To Target RED", group = "Concept")

public class VuforiaDriveToTargetRED extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 1.0; //  this is how close the camera should get to the target (inches)
                                         //  The GAIN constants set the relationship between the measured position error,
                                         //  and how much power is applied to the drive motors.  Drive = Error * Gain
                                         //  Make these values smaller for smoother control.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MM_PER_INCH = 25.40 ;   //  Metric conversion

  

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQWTtTT/////AAABmZ43cq9oLEqCsYLTNae93iRvxusM273RLfl+qHc2V2BmVwDpmWfk9zozi2LgF1cDeZTegnSEHME7E7/hzQampe0E/lFjcItCI3w9+Jln18UHS3VaBi2LzOSGc/PNlkUr35ZFwqmOiaQZ/Infop0HidwIfmB9a/jWvUNwvEZJt5SkIla1VQTyUMI4FM0IWw+p1xqc+jwORpkEW+lxovHHSxJp1rb9Fg1O7DE61prmR9d11KaAOhdokxUh7Ju3JlQG/XlQMrHxEv2R9VfMOFxP2UvkLEL5CDK7hvF96XoXEfI3CVZV+qLe9frT5PZ7YARMgm1KwxEsuKwMFoxSby++/ZRQQ8+of0xqBEdSMNaGfJ1y";
//the coment
    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    String targetName           = "";

    private DcMotor left   = null;
    private DcMotor right  = null;
    private DcMotor arm1 = null;
    private DcMotor arm2 = null;

    @Override public void runOpMode()
    {
        
  
        /*
        
        
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * To get an on-phone camera preview, use the code below.
         * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
         */
         

        
       

        
        telemetry.clear(); telemetry.update();
        
        
        
        
        
         
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        targetsFreightFrenzy.get(0).setName("Blue Storage");
        targetsFreightFrenzy.get(1).setName("Blue Alliance Wall");
        targetsFreightFrenzy.get(2).setName("Red Storage");
        targetsFreightFrenzy.get(3).setName("Red Alliance Wall");

        // Start tracking targets in the background
        targetsFreightFrenzy.activate();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left  = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        
        telemetry.update();

        waitForStart();

        boolean targetFound     = false;    // Set to true when a target is detected by Vuforia
        double  targetRange     = 0;        // Distance from camera to target in Inches
        double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
        double  drive           = 0;        // Desired forward power (-1 to +1)
        double  turn            = 0;// Desired turning power (-1 to +1)
        
       

        while (opModeIsActive())
        {
           //waitForStart();
           
             
      // Read dimensionalized data from the gyro. This gyro can report angular velocities
      // about all three axes. Additionally, it internally integrates the Z axis to
      // be able to report an absolute angular Z orientation.
         
          
              //use for delay MAYBE work
                /*if(getRuntime()<10){
                    continue;
                }*/
    
         
            //to be fixed (do not touch for now)
            /*for(int i = 0; i <3; i++){
                arm1.setPower(1);
                arm2.setPower(-1);
        
                }
            for(int i = 0; i <25; i++){
                arm1.setPower(0.1);
                arm2.setPower(0.3);
                }*/
            
        
            if(getRuntime()<3){
                left.setPower(-5);
                right.setPower(-5);
                }
            if(getRuntime()>4 && getRuntime()<5.2){
                left.setPower(1);
                right.setPower(-1);
                }
                
            
            
            
            
            
            // Look for first visible target, and save its pose.
            targetFound = false;
            for (VuforiaTrackable trackable : targetsFreightFrenzy)
            {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                {
                    targetPose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();

                    // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                    if (targetPose != null)
                    {
                        targetFound = true;
                        targetName  = trackable.getName();
                        VectorF trans = targetPose.getTranslation();

                        // Extract the X & Y components of the offset of the target relative to the robot
                        double targetX = trans.get(0) / MM_PER_INCH; // Image X axis
                        double targetY = trans.get(2) / MM_PER_INCH; // Image Z axis

                        // target range is based on distance from robot position to origin (right triangle).
                        targetRange = Math.hypot(targetX, targetY);

                        // target bearing is based on angle formed between the X axis and the target range line
                        targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

                        break;  // jump out of target tracking loop if we find a target.
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            
            
            

            // Drive to target Automatically if Left Bumper is being pressed, AND we have found a target.
            if (targetFound && getRuntime() <18) {

                // Determine heading and range error so we can use them to control the robot automatically.
                double  rangeError   = (targetRange - DESIRED_DISTANCE);
                double  headingError = targetBearing;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = -rangeError * SPEED_GAIN;
                turn  = -headingError * TURN_GAIN ;

                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
            } else if(!targetFound) {

                // drive using manual POV Joystick mode.
                
                
               
            } else{
                
            }
            telemetry.update();
            
            

            // Calculate left and right wheel powers and send to them to the motors.
            double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            left.setPower(leftPower);
            right.setPower(rightPower);

            sleep(10);
            
            telemetry.update();
        }
        
    
    }
    
}
