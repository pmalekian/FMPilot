#pragma once

#include <string>
#include <unordered_map>

#include "openpilot/cereal/gen/cpp/log.capnp.h"

inline static std::unordered_map<std::string, ParamKeyAttributes> keys = {
    {"AccessToken", {CLEAR_ON_MANAGER_START | DONT_LOG, STRING}},
    {"AdbEnabled", {PERSISTENT | BACKUP, BOOL}},
    {"AlwaysOnDM", {PERSISTENT | BACKUP, BOOL}},
    {"ApiCache_Device", {PERSISTENT, STRING}},
    {"ApiCache_FirehoseStats", {PERSISTENT, JSON}},
    {"AssistNowToken", {PERSISTENT, STRING}},
    {"AthenadPid", {PERSISTENT, INT}},
    {"AthenadUploadQueue", {PERSISTENT, JSON}},
    {"AthenadRecentlyViewedRoutes", {PERSISTENT, STRING}},
    {"BootCount", {PERSISTENT, INT}},
    {"CalibrationParams", {PERSISTENT, BYTES}},
    {"CameraDebugExpGain", {CLEAR_ON_MANAGER_START, STRING}},
    {"CameraDebugExpTime", {CLEAR_ON_MANAGER_START, STRING}},
    {"CarBatteryCapacity", {PERSISTENT, INT}},
    {"CarParams", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BYTES}},
    {"CarParamsCache", {CLEAR_ON_MANAGER_START, BYTES}},
    {"CarParamsPersistent", {PERSISTENT, BYTES}},
    {"CarParamsPrevRoute", {PERSISTENT, BYTES}},
    {"CompletedTrainingVersion", {PERSISTENT, STRING, "0"}},
    {"ControlsReady", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"CurrentBootlog", {PERSISTENT, STRING}},
    {"CurrentRoute", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, STRING}},
    {"DisableDriverCameraIR", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"DisableLogging", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"DisablePowerDown", {PERSISTENT | BACKUP, BOOL}},
    {"DisableUpdates", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"DisengageOnAccelerator", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"DongleId", {PERSISTENT, STRING}},
    {"DoReboot", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DoShutdown", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DoUninstall", {CLEAR_ON_MANAGER_START, BOOL}},
    {"DriverTooDistracted", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL}},
    {"DriverLockoutCount", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, INT, "0"}},
    {"AlphaLongitudinalEnabled", {PERSISTENT | DEVELOPMENT_ONLY | BACKUP, BOOL}},
    {"FirmwareQueryDone", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"ForcePowerDown", {PERSISTENT, BOOL}},
    {"GitBranch", {PERSISTENT, STRING}},
    {"GitCommit", {PERSISTENT, STRING}},
    {"GitCommitDate", {PERSISTENT, STRING}},
    {"GitDiff", {PERSISTENT, STRING}},
    {"GithubSshKeys", {PERSISTENT | BACKUP, STRING}},
    {"GithubUsername", {PERSISTENT | BACKUP, STRING}},
    {"GitRemote", {PERSISTENT, STRING}},
    {"GsmApn", {PERSISTENT | BACKUP, STRING}},
    {"GsmMetered", {PERSISTENT | BACKUP, BOOL, "1"}},
    {"GsmRoaming", {PERSISTENT | BACKUP, BOOL}},
    {"HardwareSerial", {PERSISTENT, STRING}},
    {"HasAcceptedTerms", {PERSISTENT, STRING, "0"}},
    {"InstallDate", {PERSISTENT, TIME}},
    {"IsDriverViewEnabled", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsEngaged", {PERSISTENT, BOOL}},
    {"IsLdwEnabled", {PERSISTENT | BACKUP, BOOL}},
    {"IsLiveStreaming", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL}},
    {"IsMetric", {PERSISTENT | BACKUP, BOOL}},
    {"IsOffroad", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsRhdDetected", {PERSISTENT, BOOL}},
    {"IsReleaseBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsTestedBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"JoystickDebugMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LanguageSetting", {PERSISTENT | BACKUP, STRING, "en"}},
    {"LastAthenaPingTime", {CLEAR_ON_MANAGER_START, INT}},
    {"LastGPSPosition", {PERSISTENT, BYTES}},
    {"LastManagerExitReason", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastOffroadStatusPacket", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, JSON}},
    {"LastAgnosPowerMonitorShutdown", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastPowerDropDetected", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastUpdateException", {CLEAR_ON_MANAGER_START, STRING}},
    {"LastUpdateRouteCount", {PERSISTENT, INT, "0"}},
    {"LastUpdateTime", {PERSISTENT, TIME}},
    {"LastUpdateUptimeOnroad", {PERSISTENT, FLOAT, "0.0"}},
    {"LiveDelay", {PERSISTENT | BACKUP, BYTES}},
    {"LiveParameters", {PERSISTENT, JSON}},
    {"LiveParametersV2", {PERSISTENT, BYTES}},
    {"LivestreamEncoderBitrate", {CLEAR_ON_MANAGER_START | DONT_LOG, INT}},
    {"LivestreamRequestKeyframe", {CLEAR_ON_MANAGER_START | DONT_LOG, BOOL}},
    {"LiveTorqueParameters", {PERSISTENT | DONT_LOG, BYTES}},
    {"LocationFilterInitialState", {PERSISTENT, BYTES}},
    {"LateralManeuverMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LongitudinalManeuverMode", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"LongitudinalPersonality", {PERSISTENT | BACKUP, INT, std::to_string(static_cast<int>(cereal::LongitudinalPersonality::STANDARD))}},
    {"NetworkMetered", {PERSISTENT | BACKUP, BOOL}},
    {"ObdMultiplexingChanged", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"ObdMultiplexingEnabled", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"Offroad_CarUnrecognized", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_ChestnutBranch", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ChestnutNotDetected", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_ChestnutOverheated", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ChestnutPcieUnavailable", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ChestnutUncompiled", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_ChestnutUpdateFailed", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_ChestnutUsbSlow", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_ConnectivityNeeded", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ConnectivityNeededPrompt", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_ExcessiveActuation", {PERSISTENT, JSON}},
    {"Offroad_NoFirmware", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_Recalibration", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_TemperatureTooHigh", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_UnregisteredHardware", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_UpdateFailed", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_DriverMonitoringUncertain", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"OnroadCycleRequested", {CLEAR_ON_MANAGER_START, BOOL}},
    {"OpenpilotEnabledToggle", {PERSISTENT | BACKUP, BOOL, "1"}},
    {"PandaHeartbeatLost", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"PrimeType", {PERSISTENT, INT}},
    {"RecordAudio", {PERSISTENT | BACKUP, BOOL}},
    {"RecordFront", {PERSISTENT | BACKUP, BOOL}},
    {"RecordFrontLock", {PERSISTENT, BOOL}},  // for the internal fleet
    {"SecOCKey", {PERSISTENT | DONT_LOG | BACKUP, STRING}},
    {"ShowDebugInfo", {PERSISTENT, BOOL}},
    {"RouteCount", {PERSISTENT, INT, "0"}},
    {"SnoozeUpdate", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL}},
    {"SshEnabled", {PERSISTENT | BACKUP, BOOL}},
    {"TermsVersion", {PERSISTENT, STRING}},
    {"TorqueBar", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"TrainingVersion", {PERSISTENT, STRING}},
    {"UbloxAvailable", {PERSISTENT, BOOL}},
    {"UpdateAvailable", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}},
    {"UpdateFailedCount", {CLEAR_ON_MANAGER_START, INT}},
    {"UpdaterAvailableBranches", {PERSISTENT, STRING}},
    {"UpdaterCurrentDescription", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterCurrentReleaseNotes", {CLEAR_ON_MANAGER_START, BYTES}},
    {"UpdaterFetchAvailable", {CLEAR_ON_MANAGER_START, BOOL}},
    {"UpdaterNewDescription", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterNewReleaseNotes", {CLEAR_ON_MANAGER_START, BYTES}},
    {"UpdaterState", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterTargetBranch", {CLEAR_ON_MANAGER_START, STRING}},
    {"UpdaterLastFetchTime", {PERSISTENT, TIME}},
    {"UptimeOffroad", {PERSISTENT, FLOAT, "0.0"}},
    {"UptimeOnroad", {PERSISTENT, FLOAT, "0.0"}},
    {"ChestnutActive", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION | CLEAR_ON_IGNITION_ON, BOOL}},
    {"ChestnutLoading", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION | CLEAR_ON_IGNITION_ON, BOOL}},
    {"ChestnutRetry", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL}},
    {"InitialCruiseSpeedKph", {PERSISTENT, INT, "40"}},
    {"NavDestination", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, JSON}},
    {"NavigationEnabled", {PERSISTENT, BOOL, "0"}},
    {"Offroad_ChestnutModelError", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_Pairing", {CLEAR_ON_MANAGER_START, JSON}},
    {"Offroad_UploadsIgnored", {CLEAR_ON_MANAGER_START, JSON}},
    {"PairingEmail", {PERSISTENT, STRING}},
    {"PairingProvider", {PERSISTENT, STRING}},

    // Jetlink: run the large driving model on a computer connected over USB
    {"JetlinkLink", {PERSISTENT | BACKUP, INT, "0"}},
    {"JetlinkProgress", {CLEAR_ON_MANAGER_START, JSON}},
    {"JetlinkSpec", {PERSISTENT, JSON}},
    {"JetlinkChargePhone", {PERSISTENT, BOOL, "0"}},
    {"Offroad_JetlinkUnavailable", {CLEAR_ON_MANAGER_START, JSON}},

    {"Version", {PERSISTENT, STRING}},

    // --- sunnypilot params --- //
    {"AccelBoostSP", {PERSISTENT | BACKUP, BOOL, "1"}},
    {"ApiCache_DriveStats", {PERSISTENT, JSON}},
    {"AutoLaneChangeBsmDelay", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"AutoLaneChangeTimer", {PERSISTENT | BACKUP, INT, "0"}},
    {"BlinkerLateralReengageDelay", {PERSISTENT | BACKUP, INT, "0"}},  // seconds
    {"BlinkerMinLateralControlSpeed", {PERSISTENT | BACKUP, INT, "20"}},  // MPH or km/h
    {"BlinkerPauseLateralControl", {PERSISTENT | BACKUP, INT, "0"}},
    {"Brightness", {PERSISTENT | BACKUP, INT, "0"}},
    {"CarList", {PERSISTENT, JSON}},
    {"CarParamsSP", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BYTES}},
    {"CarParamsSPCache", {CLEAR_ON_MANAGER_START, BYTES}},
    {"CarParamsSPPersistent", {PERSISTENT, BYTES}},
    {"CarPlatformBundle", {PERSISTENT | BACKUP, JSON}},
    {"ChevronInfo", {PERSISTENT | BACKUP, INT, "4"}},
    {"CompletedSunnylinkConsentVersion", {PERSISTENT, STRING, "0"}},
    {"CustomAccIncrementsEnabled", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"CustomAccLongPressIncrement", {PERSISTENT | BACKUP, INT, "5"}},
    {"CustomAccShortPressIncrement", {PERSISTENT | BACKUP, INT, "1"}},
    {"DeviceBootMode", {PERSISTENT | BACKUP, INT, "0"}},
    {"DevUIInfo", {PERSISTENT | BACKUP, INT, "0"}},
    {"EnableCopyparty", {PERSISTENT | BACKUP, BOOL}},
    {"OpviewEnabled", {PERSISTENT | BACKUP, BOOL}},
    {"EnableGithubRunner", {PERSISTENT | BACKUP, BOOL}},
    {"GreenLightAlert", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"HasAcceptedTermsSP", {PERSISTENT, STRING, "0"}},
    {"HideVEgoUI", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"IntelligentCruiseButtonManagement", {PERSISTENT | BACKUP , BOOL}},
    {"InteractivityTimeout", {PERSISTENT | BACKUP, INT, "0"}},
    {"IsDevelopmentBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"IsReleaseSpBranch", {CLEAR_ON_MANAGER_START, BOOL}},
    {"LaneCenteringDeadband", {PERSISTENT | BACKUP, FLOAT, "0.02"}},  // m, how far off center is close enough, 0.02 settled by road testing
    {"LaneCenteringDisplay", {PERSISTENT | BACKUP, BOOL, "0"}},  // draw the correction indicator on the onroad screen
    {"LaneCenteringE2EAuthority", {PERSISTENT | BACKUP, FLOAT, "1.0"}},  // 0 = always center, 1 = defer to a confident model
    {"LaneCenteringEnabled", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"LaneCenteringGain", {PERSISTENT | BACKUP, FLOAT, "0.60"}},  // fraction of the geometric demand applied, 0.60 settled by road testing
    {"LaneCenteringOffset", {PERSISTENT | BACKUP, FLOAT, "0.0"}},  // meters, positive is to the right
    {"LaneCenteringSingleLine", {PERSISTENT | BACKUP, BOOL, "0"}},  // keep clear of the one trusted line when the other is not trusted
    {"LaneCenteringSingleLineEdgeDist", {PERSISTENT | BACKUP, FLOAT, "1.5"}},  // m in model units, from a line at the road edge, 1.5 = normal position
    {"LaneCenteringSingleLineLaneDist", {PERSISTENT | BACKUP, FLOAT, "1.5"}},  // m in model units, from a center or lane line, 1.5 = normal position
    {"LastGPSPositionFused", {PERSISTENT, STRING}},
    {"LeadDepartAlert", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"MaxTimeOffroad", {PERSISTENT | BACKUP, INT, "1800"}},
    {"ModelRunnerTypeCache", {CLEAR_ON_ONROAD_TRANSITION, INT}},
    {"OffroadMode", {CLEAR_ON_MANAGER_START, BOOL}},
    {"Offroad_TiciSupport", {CLEAR_ON_MANAGER_START, JSON}},
    {"OnroadScreenOffBrightness", {PERSISTENT | BACKUP, INT, "0"}},
    {"OnroadScreenOffBrightnessMigrated", {PERSISTENT | BACKUP, STRING, "0.0"}},
    {"OnroadScreenOffTimer", {PERSISTENT | BACKUP, INT, "15"}},
    {"OnroadScreenOffTimerMigrated", {PERSISTENT | BACKUP, STRING, "0.0"}},
    {"OnroadUploads", {PERSISTENT | BACKUP, BOOL, "1"}},
    {"QuickBootToggle", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"QuietMode", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"RainbowMode", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"RoadEdgeLaneChangeEnabled", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"RocketFuel", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"ScreenSaverCustomText", {PERSISTENT | BACKUP, STRING, ""}},
    {"ScreenSaverEnabled", {PERSISTENT | BACKUP, BOOL, "1"}},
    {"ScreenSaverLogo", {PERSISTENT | BACKUP, STRING, ""}},
    {"ScreenSaverText", {PERSISTENT | BACKUP, INT, "0"}},
    {"ScreenSaverTimeout", {PERSISTENT | BACKUP, INT, "300"}},
    {"ShowAdvancedControls", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"ShowTurnSignals", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"StandstillTimer", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"TrueVEgoUI", {PERSISTENT | BACKUP, BOOL, "0"}},

    // MADS params
    {"Mads", {PERSISTENT | BACKUP, BOOL, "1"}},
    {"MadsMainCruiseAllowed", {PERSISTENT | BACKUP, BOOL, "1"}},
    {"MadsSteeringMode", {PERSISTENT | BACKUP, INT, "0"}},
    {"MadsUnifiedEngagementMode", {PERSISTENT | BACKUP, BOOL, "1"}},
    {"MadsMinEngageSpeed", {PERSISTENT | BACKUP, INT, "5"}},  // always stored in mph; 0 = disabled

    // Model Manager params
    {"ModelManager_ActiveBundle", {PERSISTENT, JSON}},
    {"ModelManager_ActiveBundleUSBGPU", {PERSISTENT, JSON}}, //TODO-SP: kept for migration, remove on next sync?
    {"ModelManager_ActiveBundleChestnut", {PERSISTENT, JSON}},
    {"ModelManager_ActiveJson", {CLEAR_ON_MANAGER_START, JSON}},
    {"ModelManager_ClearCache", {CLEAR_ON_MANAGER_START, BOOL}},
    {"ModelManager_DownloadRef", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, STRING}},
    {"ModelManager_Favs", {PERSISTENT | BACKUP, STRING}},
    {"ModelManager_LastSyncTime", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, INT, "0"}},
    {"ModelManager_LastSyncTime_Chestnut", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, INT, "0"}},
    {"ModelManager_ModelsCache", {PERSISTENT | BACKUP, JSON}},
    {"ModelManager_ModelsCache_Chestnut", {PERSISTENT | BACKUP, JSON}},

    // Neural Network Lateral Control
    {"NeuralNetworkLateralControl", {PERSISTENT | BACKUP, BOOL, "0"}},

    // sunnylink params
    {"EnableSunnylinkUploader", {PERSISTENT | BACKUP, BOOL}},
    {"LastSunnylinkPingTime", {CLEAR_ON_MANAGER_START, INT}},
    {"ParamsVersion", {PERSISTENT, INT}},
    {"SunnylinkCache_Roles", {PERSISTENT, STRING}},
    {"SunnylinkCache_Users", {PERSISTENT, STRING}},
    {"SunnylinkDongleId", {PERSISTENT, STRING}},
    {"SunnylinkdPid", {PERSISTENT, INT}},
    {"SunnylinkEnabled", {PERSISTENT, BOOL, "1"}},
    {"SunnylinkTempFault", {CLEAR_ON_MANAGER_START | CLEAR_ON_OFFROAD_TRANSITION, BOOL, "0"}},

    // Backup Manager params
    {"BackupManager_CreateBackup", {PERSISTENT, BOOL}},
    {"BackupManager_RestoreVersion", {PERSISTENT, STRING}},

    // sunnypilot car specific params
    {"HyundaiLongitudinalTuning", {PERSISTENT | BACKUP, INT, "0"}},
    {"SubaruStopAndGo", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"SubaruStopAndGoManualParkingBrake", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"TeslaCoopSteering", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"TeslaMadsScreenButton", {PERSISTENT | BACKUP, INT, "0"}},
    {"ToyotaEnforceStockLongitudinal", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"ToyotaStopAndGoHack", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"ToyotaTSS2Smooth", {PERSISTENT | BACKUP, BOOL, "0"}},

    {"BlindSpot", {PERSISTENT | BACKUP, BOOL, "0"}},

    // sunnypilot model params
    {"CameraOffset", {PERSISTENT | BACKUP, FLOAT, "0.0"}},
    {"LagdToggle", {PERSISTENT | BACKUP, BOOL, "1"}},
    {"LagdToggleDelay", {PERSISTENT | BACKUP, FLOAT, "0.2"}},
    {"LagdValueCache", {PERSISTENT, FLOAT, "0.2"}},
    {"LaneTurnDesire", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"LaneTurnValue", {PERSISTENT | BACKUP, FLOAT, "19.0"}},
    {"Blendful", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"PlanplusControl", {PERSISTENT | BACKUP, FLOAT, "1.0"}},

    // mapd
    {"MapAdvisorySpeedLimit", {CLEAR_ON_ONROAD_TRANSITION, FLOAT}},
    {"Mapd_ClearCache", {CLEAR_ON_MANAGER_START, BOOL}},
    {"MapdVersion", {PERSISTENT, STRING}},
    {"MapSpeedLimit", {CLEAR_ON_ONROAD_TRANSITION, FLOAT, "0.0"}},
    {"NextMapSpeedLimit", {CLEAR_ON_ONROAD_TRANSITION, JSON}},
    {"Offroad_OSMUpdateRequired", {CLEAR_ON_MANAGER_START, JSON}},
    {"OsmDbUpdatesCheck", {CLEAR_ON_MANAGER_START, BOOL}},  // mapd database update happens with device ON, reset on boot
    {"OSMDownloadBounds", {PERSISTENT, STRING}},
    {"OsmDownloadedDate", {PERSISTENT, STRING, "0.0"}},
    {"OSMDownloadLocations", {PERSISTENT, JSON}},
    {"OSMDownloadProgress", {CLEAR_ON_MANAGER_START, JSON}},
    {"OsmLocal", {PERSISTENT, BOOL}},
    {"OsmLocationName", {PERSISTENT, STRING}},
    {"OsmLocationTitle", {PERSISTENT, STRING}},
    {"OsmLocationUrl", {PERSISTENT, STRING}},
    {"OsmStateName", {PERSISTENT, STRING, "All"}},
    {"OsmStateTitle", {PERSISTENT, STRING}},
    {"OsmWayTest", {PERSISTENT, STRING}},
    {"RoadName", {CLEAR_ON_ONROAD_TRANSITION, STRING}},
    {"RoadNameToggle", {PERSISTENT | BACKUP, BOOL, "0"}},

    // Speed Limit
    {"SpeedLimitMode", {PERSISTENT | BACKUP, INT, "1"}},
    {"SpeedLimitOffsetType", {PERSISTENT | BACKUP, INT, "0"}},
    {"SpeedLimitPolicy", {PERSISTENT | BACKUP, INT, "3"}},
    {"SpeedLimitValueOffset", {PERSISTENT | BACKUP, INT, "0"}},

    // FM Pilot (fm-dev-sync). Keys of old FM settings that fm-dev-sync reshaped stay declared, never written, so
    // switching back to fm-dev-nav-settle restores the old setup; each lands with the PR that migrates it.
    {"FM_SyncMigration", {PERSISTENT, STRING}},  // the one-time migrations that ran (sunnypilot/system/fm_migration.py)
    {"FM_LaneKeepMode", {PERSISTENT | BACKUP, INT, "0"}},  // Lane Keeping: 0 Off, 1 Lane Centering (AP), 2 Lane Guard, 3 Blendful
    {"LaneGuardEnabled", {PERSISTENT | BACKUP, BOOL}},  // old Lane Guard toggle (fm-dev-set and earlier): read by the migration
    {"FM_CustomMaxTorque", {PERSISTENT | BACKUP, INT, "385"}},  // Max Torque, 385-580 step 15; 385 = stock AP; read at onroad start
    // old (fm-dev-nav-settle) torque settings, folded into Max Torque: read once by the migration, never written
    {"FM_PandaHighTorque", {PERSISTENT | BACKUP, BOOL}},
    {"FM_MaxTorque", {PERSISTENT | BACKUP, INT}},
    {"FM_TorqueFastRamp", {PERSISTENT | BACKUP, BOOL}},
    {"FM_TorqueFullCapHighAngle", {PERSISTENT | BACKUP, BOOL}},
    {"FM_TorqueMode", {PERSISTENT | BACKUP, INT}},
    {"FM_ReleaseGuardLimit", {PERSISTENT | BACKUP, INT, "0"}},  // Release Guard: 0 = Stock (off), 75, 150 counts against you
    // old (fm-dev-nav-settle) Release Guard toggle + level, now the drop-down: read once by the migration, never written
    {"FM_ReleaseGuard", {PERSISTENT | BACKUP, BOOL}},
    {"FM_ReleaseGuardMax", {PERSISTENT | BACKUP, INT}},
    {"FM_LowSpeedFFLeft", {PERSISTENT | BACKUP, INT, "0"}},   // Low-Speed Feedforward, left turns: 0 Off, 1 Gentle, 2 Medium, 3 Firm, 4 Full
    {"FM_LowSpeedFFRight", {PERSISTENT | BACKUP, INT, "0"}},  // right turns, same steps
    {"FM_LowSpeedCorrectionStrength", {PERSISTENT | BACKUP, INT, "100"}},  // 40-100 %, 100 = stock
    {"FM_LowSpeedCurvatureFloor", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"FM_StraightPullAwayFloor", {PERSISTENT | BACKUP, BOOL, "0"}},
    // old (fm-dev-nav-settle) feedforward toggle + steps and Calmer Low-Speed Correction: read once by the migration
    {"FM_LowSpeedFFEnabled", {PERSISTENT | BACKUP, BOOL}},
    {"FM_LowSpeedFFStepLeft", {PERSISTENT | BACKUP, INT}},
    {"FM_LowSpeedFFStepRight", {PERSISTENT | BACKUP, INT}},
    {"FM_LowSpeedGainEnabled", {PERSISTENT | BACKUP, BOOL}},
    {"FM_LowSpeedGainPercent", {PERSISTENT | BACKUP, INT}},

    {"FM_SmallModelSmoothingMode", {PERSISTENT | BACKUP, INT, "0"}},  // 0 Model Default, 1 0.00 s, 2 0.10 s
    {"FM_BigModelSmoothingMode", {PERSISTENT | BACKUP, INT, "0"}},
    // old (fm-dev-nav-settle) model-smoothing keys: read once by the migration
    {"FM_LateralSmoothingEnabled", {PERSISTENT | BACKUP, BOOL}},
    {"FM_LateralSmoothingSeconds", {PERSISTENT | BACKUP, FLOAT}},
    {"FM_BigModelSmoothing", {PERSISTENT | BACKUP, BOOL}},

    // Turn Hold Settings and Smooth Turn Start: same keys and meaning as on fm-dev-nav-settle, so existing values carry
    {"FM_TurnHoldEnabled", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"FM_TurnHoldPathSource", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"FM_TurnHoldDriverAdds", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"FM_TurnHoldFollowExit", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"FM_TurnRestartAfterStop", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"FM_TurnPathMinRadius", {PERSISTENT | BACKUP, FLOAT, "12.0"}},  // Turn Path Tightest Radius, 8-20 m (Hold Path Source, Turn Lead)
    {"FM_TurnPhaseIn", {PERSISTENT | BACKUP, BOOL, "0"}},  // Smooth Turn Start
    // Turn Lead Settings: same keys and meaning as on fm-dev-nav-settle, so existing values carry (Adaptive Look-Ahead
    // and Wait for the Corner defaulted on there; fresh installs start off here)
    {"FM_TurnLeadEnabled", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"FM_TurnLeadMinSpeed", {PERSISTENT | BACKUP, FLOAT, "7.0"}},  // mph, 2-7
    {"FM_TurnLeadAdaptive", {PERSISTENT | BACKUP, BOOL, "0"}},  // Adaptive Look-Ahead
    {"FM_TurnLeadGate", {PERSISTENT | BACKUP, BOOL, "0"}},  // Wait for the Corner
    {"FM_TurnLeadGateDist", {PERSISTENT | BACKUP, FLOAT, "5.0"}},  // Turn Lead Start Distance, m, 3-10
    {"FM_TurnLeadYield", {PERSISTENT | BACKUP, BOOL, "0"}},  // Turn Lead Yields to Your Hands
    // Finish Turn After Signal Cancels: same key and meaning as on fm-dev-nav-settle, so an existing value carries (on by
    // default there; fresh installs start off here)
    {"FM_TurnCarryThrough", {PERSISTENT | BACKUP, BOOL, "0"}},
    // Pre/Post Turn Behavior: same keys and meaning as on fm-dev-nav-settle (off there too), so existing values carry
    {"FM_TurnQueueGuard", {PERSISTENT | BACKUP, BOOL, "0"}},  // Queue Guard
    {"FM_TurnPulseOnPullAway", {PERSISTENT | BACKUP, BOOL, "0"}},  // Turn Pulse on Pull-Away
    {"FM_TurnSettle", {PERSISTENT | BACKUP, BOOL, "0"}},  // Settle After Turn
    {"FM_TurnSettleEarlyPush", {PERSISTENT | BACKUP, BOOL, "0"}},  // Settle After Turn: Early Right Push
    // old (fm-dev-nav-settle) Turn Hold options, folded (always on with Turn Hold): never read, never written
    {"FM_TurnNudgeEnabled", {PERSISTENT | BACKUP, BOOL}},
    {"FM_TurnHoldEarlyCorrection", {PERSISTENT | BACKUP, BOOL}},
    {"FM_TurnHoldReleaseDebounce", {PERSISTENT | BACKUP, BOOL}},
    {"FM_TurnHoldStartsAtTurnIn", {PERSISTENT | BACKUP, BOOL}},
    {"FM_TurnHoldSteeringOnly", {PERSISTENT | BACKUP, BOOL}},
    {"FM_TurnHoldSteadyPeak", {PERSISTENT | BACKUP, BOOL}},
    // old (fm-dev-nav-settle) Carry Only at Turn-In, folded (always on with Finish Turn After Signal Cancels): never read
    {"FM_TurnCarryTurnInOnly", {PERSISTENT | BACKUP, BOOL}},
    // Accel Boost picker: 0 Off, 1 Gas, 2 Sunny Boost, 3 EVP Set Speed, 4 AP Accel Control, 5 OP Personality. Same key as
    // fm-dev-nav-settle, where 0-3 meant the same (its default was 1 Gas; fresh installs start Off here). The picker alone
    // decides: sunnypilot's AccelBoostSP and AP's AccelerationControl toggles are hidden and not read by the planner.
    {"FM_AccelBoostMode", {PERSISTENT | BACKUP, INT, "0"}},
    {"FM_AccelBoostIndicator", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"FM_OpBoostCurveHold", {PERSISTENT | BACKUP, BOOL, "1"}},  // OP Personality: no boost in curves, as openpilot (on)  // Boost bar on the comma 4 side panel (same key)
    {"FM_TopReadout", {PERSISTENT | BACKUP, INT, "0"}},  // 0 Off, 1 Torque, 2 CPU Avg, 3 CPU Busiest, 4 Lead
    // Lead (Cruise): same keys and meanings as fm-dev-nav-settle (stock/off there too), so stored values carry over.
    // Confidence in percent, 25-50; 50 = openpilot's fixed cutoff
    {"FM_RadarStationaryGuard", {PERSISTENT | BACKUP, BOOL, "0"}},  // Ignore Stationary Radar Ghosts
    {"FM_RadarLeadConfidence", {PERSISTENT | BACKUP, INT, "50"}},  // Radar-matched Lead Confidence
    {"FM_CameraLeadConfidence", {PERSISTENT | BACKUP, INT, "50"}},  // Camera-only Lead Confidence
    {"FM_SmoothLeadHandoffs", {PERSISTENT | BACKUP, BOOL, "0"}},  // Smooth Lead Handoffs

    // Rivian
    {"RivianResumeEnabled", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"RivianEnhancedMads", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"RivianOfftrackCompensationEnabled", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"RivianOfftrackCompensationGain", {PERSISTENT | BACKUP, FLOAT, "0.5"}},

    // Stop Distance
    {"SPStopDistance",            {PERSISTENT | BACKUP, INT, "60"}},
    {"SPStopDistancePersonality", {PERSISTENT | BACKUP, BOOL, "0"}},

    // Following Distance
    {"SPFollowingTimeOffset", {PERSISTENT | BACKUP, INT, "0"}},

    // Acceleration Control
    {"AccelerationControl", {PERSISTENT | BACKUP, BOOL, "0"}},

    // Smart Cruise Control
    {"MapTargetVelocities", {CLEAR_ON_ONROAD_TRANSITION, STRING}},
    {"SmartCruiseControlMap", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"SmartCruiseControlVision", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"CurveSpeedControl", {PERSISTENT | BACKUP, BOOL, "0"}},

    // Torque lateral control custom params
    {"CustomTorqueParams", {PERSISTENT | BACKUP , BOOL}},
    {"EnforceTorqueControl", {PERSISTENT | BACKUP, BOOL}},
    {"LateralJerkTorqueController", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"LiveTorqueParamsToggle", {PERSISTENT | BACKUP , BOOL}},
    {"LiveTorqueParamsRelaxedToggle", {PERSISTENT | BACKUP , BOOL}},
    {"TorqueControlTune", {PERSISTENT | BACKUP, FLOAT, "0.0"}},
    {"TorqueParamsOverrideEnabled", {PERSISTENT | BACKUP, BOOL, "0"}},
    {"TorqueParamsOverrideFriction", {PERSISTENT | BACKUP, FLOAT, "0.1"}},
    {"TorqueParamsOverrideLatAccelFactor", {PERSISTENT | BACKUP, FLOAT, "2.5"}},
};
