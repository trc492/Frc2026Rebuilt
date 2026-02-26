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
        public static final boolean useLED                      = false;
        public static final boolean useRumble                   = false;
        public static final boolean hasDriverGameController     = true;
        public static final boolean hasOperatorGameController   = robotType == RobotType.RebuiltRobot;
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean showVisionStatus            = true;
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
        public static final boolean showSubsystems              = true;
        public static final boolean showSubsystemGraphs         = true;
        public static final String testSubsystemName            = Shooter.Params.TURRET_MOTOR_NAME;
        // Drive Base Subsystem
        public static final boolean useDriveBase                = true;
        public static final boolean showDriveBaseStatus         = true;
        public static final boolean debugDriveBase              = false;
        public static final boolean debugPidDrive               = false;
        public static final boolean showDrivePower              = true;
        public static final boolean useGyroAssist               = false;
        public static final boolean useAntiTipping              = false;
        // Other Subsystems
        public static final boolean useLeftShooter              = true;
        public static final boolean useRightShooter             = false;
        public static final boolean showShooterStatus           = true;
        public static final Interpolation shooterInterpolation  = Interpolation.PolynomialRegression;
        public static final boolean useMotionCompensation       = true;
        public static final boolean useIntake                   = true;
        public static final boolean showIntakeStatus            = true;
        public static final boolean useClimber                  = false;
        public static final boolean showClimberStatus           = true;
        // Auto Tasks
        public static final boolean useAutoShootTask            = true;
        public static final boolean useAutoPickupTask           = false;
        public static final boolean useAutoClimbTask            = false;
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
        public static final int CANID_FEEDER_MOTOR              = 47;   //Purple
        // Intake CAN IDs
        public static final int CANID_INTAKE_MOTOR              = 48;   //Gray
        public static final int CANID_INTAKE_DEPLOYER_MOTOR     = 49;   //White
        // Climber CAN IDs
        public static final int CANID_CLIMBER_MOTOR             = 57;   //Purple

        // Analog Input ports.
        public static final int AIN_ULTRASONIC                  = 0;
        public static final int AIN_PRESSURE_SENSOR             = 0;

        // Digital Input/Output ports.
        public static final int DIO_INTAKE_BACK_SENSOR          = 0;

        // PWM channels.
        public static final int NUM_LEDS                        = 150;
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
        public static final String TEAM_FOLDER_PATH             = "/home/lvuser/trc492";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/SteerZeroCalibration.txt";
        public static final String FIELD_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/FieldZeroCalibration.txt";
        public static final String ROBOT_CODEBASE               = "2026Rebuilt";
        public static final double ROBOT_WIDTH                  = 22.249;
        public static final double ROBOT_LENGTH                 = 22.249;
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
        //
        // Field configuration and dimensions in inches.
        //
        public static final boolean mirroredField               = false;
        public static final double fieldWidth                   = FrcField.getFieldWidth();
        public static final double fieldLength                  = FrcField.getFieldLength();
        public static final double halfFieldWidth               = fieldWidth / 2.0;
        public static final double halfFieldLength              = fieldLength / 2.0;
        public static final double[] fieldLengthTriggerPoints   = new double[]
        {
            182.11-30.0, 182.11+30.0, halfFieldLength, fieldLength - (182.11+30.0), fieldLength - (182.11-30.0)
        };
        public static final double[] fieldWidthTriggerPoints    = new double[]
        {
            halfFieldWidth-91.0, halfFieldWidth, halfFieldWidth+91.0
        };
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
        public static final double STARTPOS_BLUE_Y              = 156.61 - Robot.ROBOT_LENGTH / 2.0; // TODO: Adjust +/- 2.0 inches based on robot starting positiom
        public static final double STARTPOS_OUTPOST_X           = -26.22;
        public static final double STARTPOS_CENTER_X            = -fieldWidth / 2.0;
        public static final double STARTPOS_DEPOT_X             = -fieldWidth + 26.22;
        public static final TrcPose2D STARTPOS_BLUE_OUTPOST     = new TrcPose2D(STARTPOS_OUTPOST_X, STARTPOS_BLUE_Y, 0.0);
        public static final TrcPose2D STARTPOS_BLUE_CENTER      = new TrcPose2D(STARTPOS_CENTER_X, STARTPOS_BLUE_Y, 0.0);
        public static final TrcPose2D STARTPOS_BLUE_DEPOT       = new TrcPose2D(STARTPOS_DEPOT_X, STARTPOS_BLUE_Y, 0.0);
        public static final TrcPose2D[] blueStartPoses          =
        {
            STARTPOS_BLUE_OUTPOST, STARTPOS_BLUE_CENTER, STARTPOS_BLUE_DEPOT
        };
        //
        // Robot field positions.
        //
        public static final TrcPose2D BLUE_HUB_POSE             = new TrcPose2D(-158.32, 181.56, 0.0);
        public static final TrcPose2D BLUE_PASSBACK_AUDIENCE_SIDE =
            new TrcPose2D(0.0, 0.0, 0.0);
        public static final TrcPose2D BLUE_PASSBACK_SCORETABLE_SIDE =
            new TrcPose2D(0.0, 0.0, 0.0);
        public static final TrcPose2D BLUE_OUTPOST_PICKUP_POSE  =
            new TrcPose2D(-26.22, (30.0/2.0) + 10.0, -180.0); // TODO: Fine tune x and y
        public static final TrcPose2D BLUE_DEPOT_PICKUP_POSE    =
            new TrcPose2D(-fieldWidth + 40.0, Robot.ROBOT_WIDTH / 2.0, -90.0); // TODO: Fine tune x and y
        public static final TrcPose2D BLUE_OUTPOST_NEUTRAL_PICKUP_POSE =
            new TrcPose2D(-30.0, fieldLength / 2.0, 90.0); // TODO: Fine tune x and y
        public static final TrcPose2D BLUE_DEPOT_NEUTRAL_PICKUP_POSE =
            new TrcPose2D(-fieldWidth + 30.0, fieldLength / 2.0, -90.0); // TODO: Fine tune x and y

        public static final TrcPose2D BLUE_CLIMB_LOOKOUT_POSE           =
            new TrcPose2D(170.22, 65.0, -180.0); // TODO: Fine tune x and y
        public static final TrcPose2D BLUE_DEPOT_CLIMB_POSE             =
            new TrcPose2D(0.0, 0.0, 270.0); // TODO: Determine x and y
        public static final TrcPose2D BLUE_OUTPOST_CLIMB_POSE             =
            new TrcPose2D(0.0, 0.0, 90.0); // TODO: Determine x and y
    }   //class Game

}   //class RobotParams
