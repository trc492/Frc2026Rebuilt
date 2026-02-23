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

import frclib.driverio.FrcDashboard;

/**
 * This class contains Dashboard constants and parameters.
 */
public class Dashboard
{
    // Preferences.
    public static final String DBKEY_PREFERENCE_COMMSTATUS_MONITOR  = "Preferences/CommStatusMonitor";
    public static final String DBKEY_PREFERENCE_UPDATE_DASHBOARD    = "Preferences/UpdateDashboard";
    public static final String DBKEY_PREFERENCE_DRIVEBASE_STATUS    = "Preferences/DriveBaseStatus";
    public static final String DBKEY_PREFERENCE_DEBUG_DRIVEBASE     = "Preferences/DebugDriveBase";
    public static final String DBKEY_PREFERENCE_DEBUG_PIDDRIVE      = "Preferences/DebugPidDrive";
    public static final String DBKEY_PREFERENCE_VISION_STATUS       = "Preferences/VisionStatus";
    public static final String DBKEY_PREFERENCE_SUBSYSTEM_STATUS    = "Preferences/SubsystemStatus";

    // Drive Base.
    public static final String DBKEY_ROBOT_POSE                     = "DriveBase/RobotPose";
    public static final String DBKEY_DRIVE_ENC                      = "DriveBase/DriveEnc";
    public static final String DBKEY_STEER_FRONT                    = "DriveBase/SteerFront";
    public static final String DBKEY_STEER_BACK                     = "DriveBase/SteerBack";
    public static final String DBKEY_XPID_INFO                      = "DriveBase/XPidInfo";
    public static final String DBKEY_YPID_INFO                      = "DriveBase/YPidInfo";
    public static final String DBKEY_TURNPID_INFO                   = "DriveBase/TurnPidInfo";

    // Vision.
    public static final String DBKEY_VISION_RELOCALIZE              = "Vision/Relocalize";
    public static final String DBKEY_SHOOTER_DISTANCE_TO_TARGET     = "Vision/DistanceToTarget";

    // Shooter.
    public static final String DBKEY_SHOOTER_SHOW_STATUS            = "Shooter/ShowStatus";
    public static final String DBKEY_SHOOTER_SHOW_GRAPHS            = "Shooter/ShowGraphs";
    public static final String DBKEY_SHOOTER_USE_REGRESSION         = "Shooter/UseRegression";
    public static final String DBKEY_SHOOTER_USE_MOTION_COMPENSATION= "Shooter/UseMotionCompensation";

    public static final String DBKEY_LSHOOTER_POWER                 = "Shooter/LShooterPower";
    public static final String DBKEY_LSHOOTER_CURRENT               = "Shooter/LShooterCurrent";
    public static final String DBKEY_LSHOOTER_RPM                   = "Shooter/LShooterRPM";
    public static final String DBKEY_LSHOOTER_TARGET_RPM            = "Shooter/LShooterTargetRPM";
    public static final String DBKEY_LTILT_POWER                    = "Shooter/LTiltPower";
    public static final String DBKEY_LTILT_CURRENT                  = "Shooter/LTiltCurrent";
    public static final String DBKEY_LTILT_POS                      = "Shooter/LTiltPos";
    public static final String DBKEY_LTILT_TARGET                   = "Shooter/LTiltTarget";
    public static final String DBKEY_LXFER_POWER                    = "Shooter/LXferPower";
    public static final String DBKEY_LXFER_CURRENT                  = "Shooter/LXferCurrent";
    public static final String DBKEY_LXFER_SENSOR                   = "Shooter/LXferSensor";
    public static final String DBKEY_LXFER_ACTIVE                   = "Shooter/LXferActive";

    public static final String DBKEY_RSHOOTER_POWER                 = "Shooter/RShooterPower";
    public static final String DBKEY_RSHOOTER_CURRENT               = "Shooter/RShooterCurrent";
    public static final String DBKEY_RSHOOTER_RPM                   = "Shooter/RShooterRPM";
    public static final String DBKEY_RSHOOTER_TARGET_RPM            = "Shooter/RShooterTargetRPM";
    public static final String DBKEY_RTILT_POWER                    = "Shooter/RTiltPower";
    public static final String DBKEY_RTILT_CURRENT                  = "Shooter/RTiltCurrent";
    public static final String DBKEY_RTILT_POS                      = "Shooter/RTiltPos";
    public static final String DBKEY_RTILT_TARGET                   = "Shooter/RTiltTarget";
    public static final String DBKEY_RXFER_POWER                    = "Shooter/RXferPower";
    public static final String DBKEY_RXFER_CURRENT                  = "Shooter/RXferCurrent";
    public static final String DBKEY_RXFER_SENSOR                   = "Shooter/RXferSensor";
    public static final String DBKEY_RXFER_ACTIVE                   = "Shooter/RXferActive";

    public static final String DBKEY_TURRET_POWER                   = "Shooter/TurretPower";
    public static final String DBKEY_TURRET_CURRENT                 = "Shooter/TurretCurrent";
    public static final String DBKEY_TURRET_POS                     = "Shooter/TurretPos";
    public static final String DBKEY_TURRET_TARGET                  = "Shooter/TurretTarget";

    public static final String DBKEY_FEEDER_POWER                   = "Shooter/FeederPower";
    public static final String DBKEY_FEEDER_CURRENT                 = "Shooter/FeederCurrent";

    // Intake.
    public static final String DBKEY_INTAKE_SHOW_STATUS             = "Intake/ShowStatus";
    public static final String DBKEY_INTAKE_SHOW_GRAPHS             = "Intake/ShowGraphs";

    public static final String DBKEY_INTAKE_POWER                   = "Intake/IntakePower";
    public static final String DBKEY_INTAKE_CURRENT                 = "Intake/IntakeCurrent";
    public static final String DBKEY_INTAKE_AUTO                    = "Intake/IntakeAuto";

    public static final String DBKEY_DEPLOYER_POWER                 = "Intake/DeployerPower";
    public static final String DBKEY_DEPLOYER_CURRENT               = "Intake/DeployerCurrent";
    public static final String DBKEY_DEPLOYER_POS                   = "Intake/DeployerPos";
    public static final String DBKEY_DEPLOYER_TARGET                = "Intake/DeployerTarget";

    // Climber.
    public static final String DBKEY_CLIMBER_SHOW_STATUS            = "Climber/ShowStatus";
    public static final String DBKEY_CLIMBER_SHOW_GRAPHS            = "Climber/ShowGraphs";

    public static final String DBKEY_CLIMBER_POWER                  = "Climber/ClimberPower";
    public static final String DBKEY_CLIMBER_CURRENT                = "Climber/ClimberCurrent";
    public static final String DBKEY_CLIMBER_POS                    = "Climber/ClimberPos";
    public static final String DBKEY_CLIMBER_TARGET                 = "Climber/ClimberTarget";

    // Autonomous choices.
    public static final String DBKEY_AUTO_ALLIANCE                  = "Auto/Alliance";              //Choices
    public static final String DBKEY_AUTO_STRATEGY                  = "Auto/Strategy";              //Choices
    public static final String DBKEY_AUTO_START_POS                 = "Auto/StartPos";              //Choices
    public static final String DBKEY_AUTO_START_DELAY               = "Auto/StartDelay";            //Number

    public static final String DBKEY_AUTO_DEPOT_PICKUP              = "Auto/DepotPickup";           //Boolean
    public static final String DBKEY_AUTO_OUTPOST_PICKUP            = "Auto/OutpostPickup";         //Boolean
    public static final String DBKEY_AUTO_NEUTRAL_ZONE_PICKUP       = "Auto/NeutralZonePickup";     //Boolean
    public static final String DBKEY_AUTO_MOVE_TO                   = "Auto/MoveTo";                //Choices
    public static final String DBKEY_AUTO_PASS_BACK                 = "Auto/PassBack";              //Choices
    public static final String DBKEY_AUTO_CLIMB                     = "Auto/Climb";                 //Boolean
    public static final String DBKEY_AUTO_CLIMB_SIDE                = "Auto/ClimbSide";             //Choices

    public static final String DBKEY_AUTO_PATHFILE                  = "Auto/PathFile";              //String
    public static final String DBKEY_AUTO_X_DRIVE_DISTANCE          = "Auto/XDriveDistance";        //Number
    public static final String DBKEY_AUTO_Y_DRIVE_DISTANCE          = "Auto/YDriveDistance";        //Number
    public static final String DBKEY_AUTO_TURN_ANGLE                = "Auto/TurnAngle";             //Number
    public static final String DBKEY_AUTO_DRIVE_TIME                = "Auto/DriveTime";             //Number
    public static final String DBKEY_AUTO_DRIVE_POWER               = "Auto/DrivePower";            //Number

    // TeleOp.
    public static final String DBKEY_TELEOP_DRIVE_MODE              = "TeleOp/DriveMode";           //Choices
    public static final String DBKEY_TELEOP_DRIVE_ORIENTATION       = "TeleOp/DriveOrientation";    //Choices
    public static final String DBKEY_TELEOP_DRIVE_NORMAL_SCALE      = "TeleOp/DriveNormalScale";    //Number
    public static final String DBKEY_TELEOP_DRIVE_SLOW_SCALE        = "TeleOp/DriveSlowScale";      //Number
    public static final String DBKEY_TELEOP_TURN_NORMAL_SCALE       = "TeleOp/TurnNormalScale";     //Number
    public static final String DBKEY_TELEOP_TURN_SLOW_SCALE         = "TeleOp/TurnSlowScale";       //Number
    public static final String DBKEY_TELEOP_SHOW_DRIVE_POWER        = "TeleOp/ShowDrivePower";      //Boolean
    public static final String DBKEY_TELEOP_DRIVE_POWER             = "TeleOp/DrivePower";          //String

    // Test choices.
    public static final String DBKEY_TEST_TESTS                     = "Test/Tests";
    public static final String DBKEY_TEST_X_TARGET                  = "Test/XTarget";
    public static final String DBKEY_TEST_Y_TARGET                  = "Test/YTarget";
    public static final String DBKEY_TEST_TURN_TARGET               = "Test/TurnTarget";
    public static final String DBKEY_TEST_DRIVE_POWER               = "Test/DrivePower";
    public static final String DBKEY_TEST_TURN_POWER                = "Test/TurnPower";
    public static final String DBKEY_TEST_DRIVE_TIME                = "Test/DriveTime";
    public static final String DBKEY_TEST_X_KP                      = "Test/XKp";
    public static final String DBKEY_TEST_X_KI                      = "Test/XKi";
    public static final String DBKEY_TEST_X_KD                      = "Test/XKd";
    public static final String DBKEY_TEST_X_KF                      = "Test/XKf";
    public static final String DBKEY_TEST_X_IZONE                   = "Test/XIZone";
    public static final String DBKEY_TEST_Y_KP                      = "Test/YKp";
    public static final String DBKEY_TEST_Y_KI                      = "Test/YKi";
    public static final String DBKEY_TEST_Y_KD                      = "Test/YKd";
    public static final String DBKEY_TEST_Y_KF                      = "Test/YKf";
    public static final String DBKEY_TEST_Y_IZONE                   = "Test/YIZone";
    public static final String DBKEY_TEST_TURN_KP                   = "Test/TurnKp";
    public static final String DBKEY_TEST_TURN_KI                   = "Test/TurnKi";
    public static final String DBKEY_TEST_TURN_KD                   = "Test/TurnKd";
    public static final String DBKEY_TEST_TURN_KF                   = "Test/TurnKf";
    public static final String DBKEY_TEST_TURN_IZONE                = "Test/TurnIZone";

    public static final String DBKEY_TEST_SUBSYSTEM_NAME            = "Test/SubsystemName";
    public static final String DBKEY_TEST_SUBSYSTEM_KP              = "Test/SubsystemKp";
    public static final String DBKEY_TEST_SUBSYSTEM_KI              = "Test/SubsystemKi";
    public static final String DBKEY_TEST_SUBSYSTEM_KD              = "Test/SubsystemKd";
    public static final String DBKEY_TEST_SUBSYSTEM_KF              = "Test/SubsystemKf";
    public static final String DBKEY_TEST_SUBSYSTEM_IZONE           = "Test/SubsystemIZone";
    public static final String DBKEY_TEST_SUBSYSTEM_TOLERANCE       = "Test/SubsystemTolerance";
    public static final String DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID    = "Test/SubsystemSoftwarePid";
    public static final String DBKEY_TEST_SUBSYSTEM_TARGET_PARAM    = "Test/SubsystemTargetParam";
    public static final String DBKEY_TEST_SUBSYSTEM_KS              = "Test/SubsystemKs";
    public static final String DBKEY_TEST_SUBSYSTEM_KV              = "Test/SubsystemKv";
    public static final String DBKEY_TEST_SUBSYSTEM_KA              = "Test/SubsystemKa";

    public static final String DBKEY_TEST_SUBSYSTEM_INPUT           = "Test/SubsystemInput";
    public static final String DBKEY_TEST_SUBSYSTEM_TARGET          = "Test/SubsystemTarget";

    public static final String DBKEY_TEST_MAX_VELOCITY              = "Test/MaxVelocity";
    public static final String DBKEY_TEST_MAX_ACCELERATION          = "Test/MaxAcceleration";
    public static final String DBKEY_TEST_MAX_DECELERATION          = "Test/MaxDeceleration";
    public static final String DBKEY_TEST_ROBOT_VEL                 = "Test/RobotVelocity";
    public static final String DBKEY_TEST_TARGET_VEL                = "Test/TargetVelocity";
    public static final String DBKEY_TEST_ROBOT_POS                 = "Test/RobotPosition";
    public static final String DBKEY_TEST_TARGET_POS                = "Test/TargetPosition";

    public static final String DBKEY_TEST_RSHOOTER_TARGET_RPM       = "Test/RShooterTargetRPM";
    public static final String DBKEY_TEST_LSHOOTER_TARGET_RPM       = "Test/LShooterTargetRPM";

    public static final String DBKEY_TEST_RTILT_TARGET              = "Test/RTiltTarget";
    public static final String DBKEY_TEST_LTILT_TARGET              = "Test/LTiltTarget";

    private static FrcDashboard dashboard;

    /**
     * Constructor: Creates an instance of the object and publishes the keys in the Network Table.
     */
    public Dashboard()
    {
        dashboard = FrcDashboard.getInstance();
        // Preferences.
        dashboard.refreshKey(DBKEY_PREFERENCE_COMMSTATUS_MONITOR, RobotParams.Preferences.useCommStatusMonitor);
        dashboard.refreshKey(DBKEY_PREFERENCE_UPDATE_DASHBOARD, RobotParams.Preferences.updateDashboard);
        dashboard.refreshKey(DBKEY_PREFERENCE_DRIVEBASE_STATUS, RobotParams.Preferences.showDriveBaseStatus);
        dashboard.refreshKey(DBKEY_PREFERENCE_DEBUG_DRIVEBASE, RobotParams.Preferences.debugDriveBase);
        dashboard.refreshKey(DBKEY_PREFERENCE_DEBUG_PIDDRIVE, RobotParams.Preferences.debugPidDrive);
        dashboard.refreshKey(DBKEY_PREFERENCE_VISION_STATUS, RobotParams.Preferences.showVisionStatus);
        dashboard.refreshKey(DBKEY_PREFERENCE_SUBSYSTEM_STATUS, RobotParams.Preferences.showSubsystems);
        // Drive Base.
        dashboard.refreshKey(DBKEY_ROBOT_POSE, "");
        dashboard.refreshKey(DBKEY_DRIVE_ENC, "");
        dashboard.refreshKey(DBKEY_STEER_FRONT, "");
        dashboard.refreshKey(DBKEY_STEER_BACK, "");
        dashboard.refreshKey(DBKEY_XPID_INFO, "");
        dashboard.refreshKey(DBKEY_YPID_INFO, "");
        dashboard.refreshKey(DBKEY_TURNPID_INFO, "");
        // Vision.
        dashboard.refreshKey(DBKEY_VISION_RELOCALIZE, RobotParams.Preferences.visionRelocalizeEnabled);
        dashboard.refreshKey(DBKEY_SHOOTER_DISTANCE_TO_TARGET, 0.0);
        // Shooter.
        dashboard.refreshKey(DBKEY_SHOOTER_SHOW_STATUS, RobotParams.Preferences.showShooterStatus);
        dashboard.refreshKey(DBKEY_SHOOTER_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs);
        dashboard.refreshKey(DBKEY_LSHOOTER_RPM, 0.0);
        dashboard.refreshKey(DBKEY_LSHOOTER_TARGET_RPM, 0.0);
        dashboard.refreshKey(DBKEY_RSHOOTER_RPM, 0.0);
        dashboard.refreshKey(DBKEY_RSHOOTER_TARGET_RPM, 0.0);
        dashboard.refreshKey(DBKEY_LTILT_POS, 0.0);
        dashboard.refreshKey(DBKEY_LTILT_TARGET, 0.0);
        dashboard.refreshKey(DBKEY_RTILT_POS, 0.0);
        dashboard.refreshKey(DBKEY_RTILT_TARGET, 0.0);
        // Intake.
        dashboard.refreshKey(DBKEY_INTAKE_SHOW_STATUS, RobotParams.Preferences.showIntakeStatus);
        dashboard.refreshKey(DBKEY_INTAKE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs);
        // Climber.
        dashboard.refreshKey(DBKEY_CLIMBER_SHOW_STATUS, RobotParams.Preferences.showClimberStatus);
        dashboard.refreshKey(DBKEY_CLIMBER_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs);
        // TeleOp.
        dashboard.refreshKey(DBKEY_TELEOP_DRIVE_NORMAL_SCALE, FrcTeleOp.DEF_DRIVE_NORMAL_SCALE);
        dashboard.refreshKey(DBKEY_TELEOP_DRIVE_SLOW_SCALE, FrcTeleOp.DEF_DRIVE_SLOW_SCALE);
        dashboard.refreshKey(DBKEY_TELEOP_TURN_NORMAL_SCALE, FrcTeleOp.DEF_TURN_NORMAL_SCALE);
        dashboard.refreshKey(DBKEY_TELEOP_TURN_SLOW_SCALE, FrcTeleOp.DEF_TURN_SLOW_SCALE);
        dashboard.refreshKey(DBKEY_TELEOP_SHOW_DRIVE_POWER, RobotParams.Preferences.showDrivePower);
        dashboard.refreshKey(DBKEY_TELEOP_DRIVE_POWER, "");
    }   //Dashboard
    /**
     * This method returns the FrcDashboard object.
     *
     * @return dashboard object.
     */
    public FrcDashboard getDashboard()
    {
        return dashboard;
    }   //getDashboard

    /**
     * This method is called periodically to check the Dashboard switch for enabling/disabling Dashboard update.
     */
    public static void checkDashboardUpdateEnabled()
    {
        boolean updateDashboard = dashboard.getBoolean(
            Dashboard.DBKEY_PREFERENCE_UPDATE_DASHBOARD, RobotParams.Preferences.updateDashboard);
        boolean updateEnabled = dashboard.isDashboardUpdateEnabled();

        if (!updateEnabled && updateDashboard)
        {
            dashboard.enableDashboardUpdate(1, true);
        }
        else if (updateEnabled && !updateDashboard)
        {
            dashboard.disableDashboardUpdate();
        }
    }   //checkDashboardUpdateEnabled

}   //class Dashboard
