/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frclib.robotcore.FrcField;
import teamcode.subsystems.DriveBase.RobotType;
import teamcode.subsystems.Shooter;
import trclib.dataprocessor.TrcLookupTable.Interpolation;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final RobotType robotType                 = RobotType.RebuiltRobot;
        public static final boolean inCompetition               = false;
        public static final boolean hybridMode                  = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useCommStatusMonitor        = false;
        // Sensors and Indicators
        public static final boolean usePdp                      = false;
        public static final boolean usePressureSensor           = false;
        // Driver feedback
        // Status Update: Dashboard Update may affect robot loop time, don't do it when in competition.
        public static final boolean updateDashboard             = !inCompetition;   // Start up default value.
        public static final boolean useLED                      = true;
        public static final boolean useRumble                   = true;
        public static final boolean hasDriverGameController     = true;
        public static final boolean hasOperatorGameController   = robotType == RobotType.RebuiltRobot;
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean showVisionStatus            = !inCompetition;
        public static final boolean usePhotonVision             = true;
        public static final boolean useOpenCvVision             = false;
        public static final boolean useWebcamAprilTagVision     = false;
        public static final boolean useWebcamColorBlobVision    = false;
        public static final boolean useSolvePnp                 = false;
        public static final boolean useStreamCamera             = false;
        public static final boolean visionRelocalizeEnabled     = true;
        public static final boolean useWpiLibPoseEstimator      = true;
        // Master switches for Subsystems
        public static final boolean useSubsystems               = robotType == RobotType.RebuiltRobot;
        public static final boolean showSubsystemStatus         = true;
        public static final boolean zeroCalSubsystems           = true;
        public static final String testSubsystemName            = Shooter.Params.TURRET_MOTOR_NAME;
        // Drive Base Subsystem
        public static final boolean useDriveBase                = true;
        public static final boolean showDriveBaseStatus         = !inCompetition;
        public static final boolean debugDriveBase              = false;
        public static final boolean debugPidDrive               = false;
        public static final boolean showDrivePower              = false;
        public static final boolean useGyroAssist               = false;
        public static final boolean useAntiTipping              = false;
        // Other Subsystems
        public static final boolean useLeftShooter              = true;
        public static final boolean useRightShooter             = true;
        public static final boolean showShooterStatus           = true;
        public static final Interpolation shooterInterpolation  = Interpolation.PolynomialRegression;
        public static final boolean useMotionCompensation       = true;
        public static final boolean useIntake                   = true;
        public static final boolean showIntakeStatus            = true;
        public static final boolean useClimber                  = true;
        public static final boolean showClimberStatus           = true;
        // Auto Tasks
        public static final boolean useAutoShootTask            = true;
        public static final boolean useAutoPickupTask           = false;
        public static final boolean useAutoClimbTask            = true;
    }   //class Preferences

    /**
     * This class contains the Robot Hardware Configurations.
     */
    public static class HwConfig
    {
        // Joystick ports.
        public static final int XBOX_DRIVER_CONTROLLER          = 0;
        public static final int XBOX_OPERATOR_CONTROLLER        = 1;
        // CAN Bus Names
        public static final String CANBUS_CANIVORE              = "2026_CANivore";
        // CAN IDs.
        public static final int CANID_PDP                       = 1;
        public static final int CANID_PCM                       = 2;
        public static final int CANID_PIGEON2                   = 10;
        // Drive Motor CAN IDs.
        public static final int CANID_FLDRIVE_MOTOR             = 3;    //Orange
        public static final int CANID_FRDRIVE_MOTOR             = 4;    //Yellow
        public static final int CANID_BLDRIVE_MOTOR             = 5;    //Green
        public static final int CANID_BRDRIVE_MOTOR             = 6;    //Blue
        // Swerve CAN IDs.
        public static final int CANID_FLSTEER_MOTOR             = 13;   //Orange
        public static final int CANID_FRSTEER_MOTOR             = 14;   //Yellow
        public static final int CANID_BLSTEER_MOTOR             = 15;   //Green
        public static final int CANID_BRSTEER_MOTOR             = 16;   //Blue
        public static final int CANID_FLSTEER_ENCODER           = 23;   //Orange
        public static final int CANID_FRSTEER_ENCODER           = 24;   //Yellow
        public static final int CANID_BLSTEER_ENCODER           = 25;   //Green
        public static final int CANID_BRSTEER_ENCODER           = 26;   //Blue
        // Left Shooter CAN IDs.
        public static final int CANID_LSHOOTER_PRIMARY_MOTOR    = 7;    //Purple
        public static final int CANID_LSHOOTER_FOLLOWER_MOTOR   = 8;    //Gray
        public static final int CANID_LSHOOTER_TILT_MOTOR       = 9;    //White
        public static final int CANID_LTRANSFER_MOTOR           = 37;   //Purple
        // Right Shooter CAN IDs.
        public static final int CANID_RSHOOTER_PRIMARY_MOTOR    = 27;   //Purple
        public static final int CANID_RSHOOTER_FOLLOWER_MOTOR   = 28;   //Gray
        public static final int CANID_RSHOOTER_TILT_MOTOR       = 29;   //White
        public static final int CANID_RTRANSFER_MOTOR           = 38;   //Gray
        // Common Shooter CAN IDs.
        public static final int CANID_TURRET_MOTOR              = 39;   //White
        public static final int CANID_TURRET_ABS_ENCODER        = 58;   //Gray
        public static final int CANID_FEEDER_MOTOR              = 47;   //Purple
        // Intake CAN IDs
        public static final int CANID_INTAKE_MOTOR              = 48;   //Gray
        public static final int CANID_INTAKE_DEPLOYER_ENCODER   = 49;   //White
        // Climber CAN IDs
        public static final int CANID_CLIMBER_MOTOR             = 57;   //Purple

        // Analog Input ports.
        public static final int AIN_ULTRASONIC                  = 0;
        public static final int AIN_PRESSURE_SENSOR             = 0;

        // Digital Input/Output ports.

        // PWM channels.
        public static final int NUM_LEDS                        = 209;
        public static final int PWM_CHANNEL_LED                 = 0;

        // Relay channels.

        // Pneumatic channels.

        // PDP Channels.
        public static final ModuleType PDP_MODULE_TYPE          = ModuleType.kRev;

        public static final double BATTERY_CAPACITY_WATT_HOUR   = 18.0*12.0;
    }   //class HwConfig

    /**
     * This class contains Robot parameters.
     */
    public static class Robot
    {
        public static final String VOL_PATH                     = "/u";
        public static final String DEF_VOL_PATH                 = "/home/lvuser";
        public static final String TEAM_FOLDER_NAME             = "/trc492";
        public static String teamFolderPath                     = VOL_PATH + TEAM_FOLDER_NAME;
        public static final String LOG_FOLDER_NAME              = "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE_NAME     = "/SteerZeroCalibration.txt";
        public static final String FIELD_ZERO_CAL_FILE_NAME     = "/FieldZeroCalibration.txt";
        public static final String ROBOT_CODEBASE               = "2026Rebuilt";
        public static final double ROBOT_WIDTH                  = 34.0;
        public static final double ROBOT_LENGTH                 = 34.0;
    }   //class Robot

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        //
        // Game time.
        //
        public static final double AUTONOMOUS_PERIOD            = 20.0;     // in seconds
        public static final double TELEOP_PERIOD                = 140.0;    // in seconds
        public static final double ENDGAME_THRESHOLD            = 30.0;     // in seconds
        public static final double SHIFT_THRESHOLD              = 2.0;
        public static final double[] SHIFTS                     = new double[] {10, 35, 60, 85, 110};
        //
        // Field configuration and dimensions in inches.
        //
        public static final boolean mirroredField               = false;
        public static final double fieldWidth                   = FrcField.getFieldWidth();     //317.69
        public static final double fieldLength                  = FrcField.getFieldLength();    //651.22
        public static final double halfFieldWidth               = fieldWidth / 2.0;             //158.845
        public static final double halfFieldLength              = fieldLength / 2.0;            //325.61
        //
        // AprilTag Poses
        //
        private static TrcPose2D[] getAprilTagFieldPoses()
        {
            TrcPose2D[] poses = new TrcPose2D[32];

            for (int i = 0; i < poses.length; i++)
            {
                poses[i] = FrcField.getAprilTagFieldPose(i + 1);
                TrcDbgTrace.globalTraceDebug("AprilTagPoses", "[%d] %s", i, poses[i]);
            }

            return poses;
        }   //getAprilTagFieldPoses

        public static final TrcPose2D[] aprilTagFieldPoses      = getAprilTagFieldPoses();
        public static final int[] blueHubAprilTags              = new int[] {10, 2, 5, 9, 11, 8, 3, 4};
        public static final int[] redHubAprilTags               = new int[] {26, 18, 21, 25, 24, 27, 19, 20};
        public static final int[] anyHubAprilTags               =
            new int[] {10, 26, 2, 18, 5, 21, 9, 25, 11, 24, 8, 27, 3, 19, 4, 20};
        public static final int[] blueTowerAprilTags            = new int[] {31, 32};
        public static final int[] redTowerAprilTags             = new int[] {15, 16};
        //
        // Robot starting positions.
        //
        public static final double STARTPOS_BLUE_SIDE_Y         = 156.61 + Robot.ROBOT_LENGTH / 2.0;    //167.7345
        public static final double STARTPOS_BLUE_CENTER_Y       = 156.61 - Robot.ROBOT_LENGTH / 2.0;    //145.4855
        public static final double STARTPOS_OUTPOST_X           = -17.22;
        public static final double STARTPOS_CENTER_X            = -fieldWidth / 2.0;                    //-158.845
        public static final double STARTPOS_DEPOT_X             = -fieldWidth - STARTPOS_OUTPOST_X;     //-300.47

        public static final TrcPose2D STARTPOS_BLUE_OUTPOST     =           // (-17.22,167.7345,-90.0)
            new TrcPose2D(STARTPOS_OUTPOST_X, STARTPOS_BLUE_SIDE_Y, -90.0);
        public static final TrcPose2D STARTPOS_BLUE_CENTER      =           // (-158.845,145.4855,180.0)
            new TrcPose2D(STARTPOS_CENTER_X, STARTPOS_BLUE_CENTER_Y, 180.0);
        public static final TrcPose2D STARTPOS_BLUE_DEPOT       =           // (-300.47,167.7345,90.0)
            new TrcPose2D(STARTPOS_DEPOT_X, STARTPOS_BLUE_SIDE_Y, 90.0);
        public static final TrcPose2D[] blueStartPoses          =
        {
            STARTPOS_BLUE_OUTPOST, STARTPOS_BLUE_CENTER, STARTPOS_BLUE_DEPOT
        };
        //
        // Robot field positions.
        //
        public static final TrcPose2D BLUE_HUB_POSE             =           // (-158.845,182.11,0.0)
            new TrcPose2D(-fieldWidth/2.0, 182.11, 0.0);
        public static final TrcPose2D BLUE_HUB_BACK_CENTER_POSE =           // (-158.845, 215.87, 0.0)
            new TrcPose2D(-fieldWidth/2.0, 182.11 + 33.76, 0.0);
        public static final TrcPose2D BLUE_PASSBACK_AUDIENCE_SIDE =         // (-238.2675,78.305,0.0)
            new TrcPose2D(-fieldWidth*3.0/4.0, 78.305, 0.0);
        public static final TrcPose2D BLUE_PASSBACK_SCORETABLE_SIDE =       // (-79.4225,78.305.0,0.0)
            new TrcPose2D(-fieldWidth/4.0, 78.305, 0.0);
        public static final TrcPose2D RED_PASSBACK_AUDIENCE_SIDE =          // (-238.2675,572.915,0.0)
            new TrcPose2D(-fieldWidth*3.0/4.0, fieldLength - 78.305, 0.0);
        public static final TrcPose2D RED_PASSBACK_SCORETABLE_SIDE =        // (-79.4225,572.915,0.0)
            new TrcPose2D(-fieldWidth/4.0, fieldLength - 78.305, 0.0);
        public static final double HUB_WIDTH                    = 60.0;     // inches
        public static final double deadZoneAngleRad             =           // 19.761664387128421576707179475099 deg
            Math.atan2(fieldWidth/2.0 - Math.abs(BLUE_PASSBACK_SCORETABLE_SIDE.x) - HUB_WIDTH/2.0,  //49.4225
                       BLUE_HUB_BACK_CENTER_POSE.y - BLUE_PASSBACK_SCORETABLE_SIDE.y);              //137.565
        public static final double deadZoneLength               = HUB_WIDTH/2.0/Math.tan(deadZoneAngleRad); //83.503465020992462947038292275785
        public static final TrcPose2D BLUE_HUB_DEAD_ZONE_CENTER_POSE =      // (-158.845, 299.373465, 0.0)
            new TrcPose2D(-fieldWidth/2.0, BLUE_HUB_BACK_CENTER_POSE.y + deadZoneLength, 0.0);
        public static final TrcPose2D BLUE_OUTPOST_PICKUP_POSE  =           // (-26.22,33.249,-180.0)
            new TrcPose2D(-26.22, Robot.ROBOT_LENGTH/2.0 - 3.0, -180.0);
        public static final TrcPose2D BLUE_DEPOT_PICKUP_POSE    =           // (-277.69,11.1245,90.0)
            new TrcPose2D(-fieldWidth + 84.0, 60.0, 180.0);
        public static final TrcPose2D BLUE_OUTPOST_NEUTRAL_PICKUP_POSE =    // (-55.89,301.61,-90.0)
            new TrcPose2D(-55.89, fieldLength / 2.0 - 24.0, -90.0);
        public static final TrcPose2D BLUE_DEPOT_NEUTRAL_PICKUP_POSE =      // (-261.8,301.61,90.0)
            new TrcPose2D(-fieldWidth + 55.89, fieldLength / 2.0 - 24.0, 90.0);

        public static final TrcPose2D BLUE_CLIMB_LOOKOUT_POSE   =
            new TrcPose2D(170.22, 65.0, -180.0); // TODO: Fine tune x and y
        public static final TrcPose2D BLUE_DEPOT_CLIMB_POSE     =
            new TrcPose2D(-190.95, 53.29, -90.0); // TODO: Determine x and y
        public static final TrcPose2D BLUE_OUTPOST_CLIMB_POSE   =
            new TrcPose2D(-105.95, 53.29, 90.0); // TODO: Determine x and y

        public static final double allianceAreaWidth            = 182.11;   // Distance from alliance wall to center of trench.
        public static final double[] fieldLengthTriggerPoints   = new double[]
        {
            // 152.11, 215.87, 299.373465, 325.61
            allianceAreaWidth-30.0, BLUE_HUB_BACK_CENTER_POSE.y, BLUE_HUB_DEAD_ZONE_CENTER_POSE.y, halfFieldLength,
            // 351.846535, 435.35
            fieldLength - BLUE_HUB_DEAD_ZONE_CENTER_POSE.y, fieldLength - BLUE_HUB_BACK_CENTER_POSE.y,
            // 499.11
            fieldLength - (allianceAreaWidth-30.0)
        };
        public static final double[] fieldWidthTriggerPoints    = new double[]
        {
            // -188.846, -158.845, -128.845
            -halfFieldWidth - HUB_WIDTH/2.0, -halfFieldWidth, -halfFieldWidth + HUB_WIDTH/2.0
        };

        // public static final TrcPose2D[] blueDoubleSweepMainDepotPath = new TrcPose2D[] {
        //     new TrcPose2D(-295.80, 281.61, 90.00),
        //     new TrcPose2D(-261.80, 301.61, 90.00),
        //     new TrcPose2D(-166.80, 301.61, 90.00),
        //     new TrcPose2D(-285.80, 301.61, 0.00),
        //     new TrcPose2D(-292.80, 143.73, 0.00),
        //     new TrcPose2D(-292.80, 257.41, 0.00),
        //     new TrcPose2D(-187.68, 268.61, 90.00),
        //     new TrcPose2D(-139.89, 246.89, 180.00),
        //     new TrcPose2D(-190.62, 230.60, -90.00),
        //     new TrcPose2D(-210.61, 227.88, -45.00),
        //     new TrcPose2D(-210.61, 60.82, -45.00)
        // };

        // public static final TrcPose2D[] blueDoubleSweepMainOutpostPath = new TrcPose2D[] {
        //     new TrcPose2D(-21.89, 281.61, -90.00),
        //     new TrcPose2D(-55.89, 301.61, -90.00),
        //     new TrcPose2D(-150.89, 301.61, -90.00),
        //     new TrcPose2D(-31.89, 301.61, 0.00),
        //     new TrcPose2D(-24.89, 143.73, 0.00),
        //     new TrcPose2D(-24.89, 257.41, 0.00),
        //     new TrcPose2D(-130.01, 268.61, -90.00),
        //     new TrcPose2D(-177.80, 246.89, -180.00),
        //     new TrcPose2D(-137.07, 230.60, 90.00),
        //     new TrcPose2D(-107.08, 227.88, 135.00),
        //     new TrcPose2D(-107.08, 119.82, 135.00)
        // };

        // public static final TrcPose2D[] blueDoubleSweepDepotTrenchPath = new TrcPose2D[] {
        //     new TrcPose2D(-295.80, 281.61, 90.00),
        //     new TrcPose2D(-261.80, 301.61, 90.00),
        //     new TrcPose2D(-166.80, 301.61, 90.00),
        //     new TrcPose2D(-285.80, 301.61, 0.00),
        //     new TrcPose2D(-292.80, 147.73, 0.00),
        //     new TrcPose2D(-292.80, 257.41, 0.00),
        //     new TrcPose2D(-187.68, 268.61, 90.00),
        //     new TrcPose2D(-139.89, 246.89, 180.00),
        //     new TrcPose2D(-186.60, 232.34, -90.00),
        //     new TrcPose2D(-235.47, 227.34, -90.00),
        //     new TrcPose2D(-275.60, 270.81, -90.00),
        //     new TrcPose2D(-290.20, 250.01, 0.00),
        //     new TrcPose2D(-293.52, 147.73, 0.00)
        // };

        // public static final TrcPose2D[] blueDoubleSweepOutpostTrenchPath = new TrcPose2D[] {
        //     new TrcPose2D(-21.89, 281.61, -90.00),
        //     new TrcPose2D(-55.89, 301.61, -90.00),
        //     new TrcPose2D(-150.89, 301.61, -90.00),
        //     new TrcPose2D(-31.89, 301.61, 0.00),
        //     new TrcPose2D(-24.89, 143.73, 0.00),
        //     new TrcPose2D(-24.89, 257.41, 0.00),
        //     new TrcPose2D(-130.01, 268.61, -90.00),
        //     new TrcPose2D(-177.80, 246.89, -180.00),
        //     new TrcPose2D(-131.09, 232.34, 90.00),
        //     new TrcPose2D(-82.22, 227.34, 90.00),
        //     new TrcPose2D(-42.09, 275.81, 90.00),
        //     new TrcPose2D(-27.49, 250.01, 0.00),
        //     new TrcPose2D(-24.17, 123.46, 0.00)
        // };

        public static final TrcPose2D[] blueDoubleSweepDepotTrenchPath = new TrcPose2D[] {
            new TrcPose2D(-295.80, 281.61, 90.00),
            new TrcPose2D(-261.80, 311.61, 110.00),
            new TrcPose2D(-166.80, 311.61, 110.00),
            new TrcPose2D(-278.80, 301.61, 0.00),
            new TrcPose2D(-284.80, 151.73, 0.00),
            new TrcPose2D(-293.58, 233.32, 0.00),
            new TrcPose2D(-249.59, 240.80, 90.00),
            new TrcPose2D(-164.12, 240.80, 90.00),
            new TrcPose2D(-119.05, 251.24, 0.00),
            new TrcPose2D(-162.93, 274.59, -90.00),
            new TrcPose2D(-253.39, 280.56, -90.00),
            new TrcPose2D(-296.84, 267.53, 0.00),
            new TrcPose2D(-296.29, 151.73, 0.00)
        };

        public static final TrcPose2D[] blueDoubleSweepOutpostTrenchPath = new TrcPose2D[] {
            new TrcPose2D(-21.89, 281.61, -90.00),
            new TrcPose2D(-55.89, 311.61, -110.00),
            new TrcPose2D(-150.89, 311.61, -110.00),
            new TrcPose2D(-38.89, 301.61, 0.00),
            new TrcPose2D(-32.89, 151.73, 0.00),
            new TrcPose2D(-24.11, 233.32, 0.00),
            new TrcPose2D(-68.10, 240.80, -90.00),
            new TrcPose2D(-153.57, 240.80, -90.00),
            new TrcPose2D(-198.64, 251.24, 0.00),
            new TrcPose2D(-154.76, 274.59, 90.00),
            new TrcPose2D(-64.30, 280.56, 90.00),
            new TrcPose2D(-17.85, 267.53, 0.00),
            new TrcPose2D(-17.40, 151.73, 0.00)
        };

        // public static final TrcPose2D[] blueDoubleSweepDepotBumpPath = new TrcPose2D[] {
        //     new TrcPose2D(-295.80, 281.61, 90.00),
        //     new TrcPose2D(-261.80, 301.61, 90.00),
        //     new TrcPose2D(-166.80, 301.61, 90.00),
        //     new TrcPose2D(-210.09, 225.17, 135.00),
        //     new TrcPose2D(-210.09, 85.10, 135.00),
        //     new TrcPose2D(-297.12, 85.10, 0.00),
        //     new TrcPose2D(-297.12, 242.00, 0.00),
        //     new TrcPose2D(-206.15, 275.07, 90.00),
        //     new TrcPose2D(-147.50, 275.98, 180.00),
        //     new TrcPose2D(-185.51, 229.51, -135.00),
        //     new TrcPose2D(-210.09, 226.26, -135.00),
        //     new TrcPose2D(-210.06, 87.94, -135.00)
        // };

        public static final TrcPose2D[] blueDoubleSweepDepotBumpPath = new TrcPose2D[] {
            new TrcPose2D(-295.80, 281.61, 90.00),
            new TrcPose2D(-261.80, 311.61, 110.00),
            new TrcPose2D(-166.80, 311.61, 110.00),
            new TrcPose2D(-278.80, 301.61, 0.00),
            new TrcPose2D(-284.80, 151.73, 0.00),
            new TrcPose2D(-287.58, 233.32, 0.00),
            new TrcPose2D(-249.59, 240.80, 90.00),
            new TrcPose2D(-164.12, 240.80, 90.00),
            new TrcPose2D(-119.05, 251.24, 0.00),
            new TrcPose2D(-162.93, 274.59, -90.00),
            new TrcPose2D(-225.93, 274.59, -90.00),
            new TrcPose2D(-225.18, 231.14, -45.00),
            new TrcPose2D(-225.09, 75.70, -45.00)
        };

        public static final TrcPose2D[] blueDoubleSweepOutpostBumpPath = new TrcPose2D[] {
            new TrcPose2D(-21.89, 281.61, -90.00),
            new TrcPose2D(-55.89, 311.61, -110.00),
            new TrcPose2D(-150.89, 311.61, -110.00),
            new TrcPose2D(-38.89, 301.61, 0.00),
            new TrcPose2D(-32.89, 151.73, 0.00),
            new TrcPose2D(-30.11, 233.32, 0.00),
            new TrcPose2D(-68.10, 240.80, -90.00),
            new TrcPose2D(-153.57, 240.80, -90.00),
            new TrcPose2D(-198.64, 251.24, 0.00),
            new TrcPose2D(-154.76, 274.59, 90.00),
            new TrcPose2D(-91.76, 274.59, 90.00),
            new TrcPose2D(-92.51, 231.14, 45.00),
            new TrcPose2D(-92.60, 75.70, 45.00)
        };
        
    }   //class Game

}   //class RobotParams
