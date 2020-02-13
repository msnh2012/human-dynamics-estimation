/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanDataCollector.h"
#include "IHumanState.h"
#include "IHumanWrench.h"
#include "IHumanDynamics.h"
#include "Utils.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include "matio.h"

// TODO: Double check if the header works on WINDOWS
#include <experimental/filesystem>
#include <sys/stat.h>

#include <chrono>
#include <ratio>
#include <algorithm>
#include <unordered_map>

const std::string DeviceName = "HumanDataCollector";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;


class HumanDataCollector::impl
{
public:

    double period;
    std::string portPrefix;
    bool firstData = true;

    hde::interfaces::IHumanState* iHumanState = nullptr;
    hde::interfaces::IHumanWrench* iHumanWrenchMeasurements = nullptr;
    hde::interfaces::IHumanWrench* iHumanWrenchEstimates = nullptr; //This interface points to HumanDynamicsEstimator, which gives offset removed wrench measurements and the estimates
    hde::interfaces::IHumanDynamics* iHumanDynamics = nullptr;

    struct {
        bool stateProvider = false;
        bool wrenchProvider = false;
        bool dynamicsEstimator = false;
    } isAttached;

    // Human data buffer structs
    struct {

        std::vector<double> time;
        std::vector<std::string> stateJointNames;
        std::vector<std::string>  wrenchMeasurementSourceNames;
        std::vector<std::string> wrenchEstimateSourceNames;
        std::vector<std::string> dynamicsJointNames;
        std::unordered_map<std::string, std::vector<std::vector<double>>> data;

    } humanDataStruct;

    // Interface data buffers

    // Base Quantities
    std::string baseName;
    std::array<double, 3> basePosition;
    std::array<double, 4> baseOrientation;
    std::array<double, 7> basePoseArray;
    std::vector<double> basePoseVec;

    std::array<double, 6> baseVelocity;
    std::vector<double> baseVeclocityVec;

    // Joint Quantities
    size_t stateNumberOfJoints;
    std::vector<std::string> stateJointNames;
    std::vector<double> jointPositionsVec;
    std::vector<double> jointVelocitiesVec;

    // CoM Quantities
    std::array<double, 3> comPosition;
    std::vector<double> comPositionVec;
    std::array<double, 3> comVelocity;
    std::vector<double> comVelocityVec;
    std::array<double, 6> comProperAccInBaseFrame;
    std::vector<double> comProperAccInBaseFrameVec;
    std::array<double, 6> comProperAccInWorldFrame;
    std::vector<double> comProperAccInWorldFrameVec;

    // Wrench Measurements
    size_t numberOfWrenchMeasurementSources;
    std::vector<std::string> wrenchMeasurementSourceNames;
    std::vector<double> wrenchMeasurementValuesVec;

    // Wrench Estimates
    size_t numberOfWrenchEstimateSources;
    std::vector<std::string> wrenchEstimateSourceNames;
    std::vector<double> wrenchEstimateValuesVec;

    // Joint Torques
    size_t dynamicsNumberOfJoints;
    std::vector<std::string> dynamicsJointNames;
    std::vector<double> jointTorquesVec;

    // Yarp ports for streaming data from IHumanState interface of HumanStateProvider
    yarp::os::BufferedPort<yarp::sig::Vector> basePoseDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> baseVelocityDataPort;
    yarp::os::BufferedPort<yarp::os::Bottle> stateJointNamesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> jointPositionsDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> jointVelocitiesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comPositionDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comVelocityDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comProperAccelerationInBaseFrameDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comProperAccelerationInWorldFrameDataPort;

    // Yarp ports for streaming data from IHumanWrench interface of HumanWrenchProvider
    yarp::os::BufferedPort<yarp::os::Bottle> wrenchMeasurementSourceNamesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> wrenchMeasurementValuesDataPort;

    // Yarp ports for streaming data from IHumanWrench interfce and IHumanDynamics interface of HumanDynamicsEstimator
    yarp::os::BufferedPort<yarp::os::Bottle> wrenchEstimateSourceNamesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> wrenchEstimateValuesDataPort;

    yarp::os::BufferedPort<yarp::os::Bottle> dynamicsJointNamesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> jointTorquesDataPort;


    // MATIO Logging
    bool matioLogger = false;
    const std::experimental::filesystem::path originalWorkingDirectory = std::experimental::filesystem::current_path();
    std::string matLogDirectory = "";
    std::string matLogFileName = "matLogFile.mat"; //TODO: Improve file handling for multiple file logging
    mat_t *matFilePtr = nullptr;
    matvar_t *matDataStruct = nullptr;

    // Time Buffers
    std::chrono::system_clock::time_point startTime;
    std::chrono::system_clock::time_point currentTime;
};

HumanDataCollector::HumanDataCollector()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanDataCollector::~HumanDataCollector() = default;

bool HumanDataCollector::open(yarp::os::Searchable &config) {

    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period";
    }

    if (!(config.check("portPrefix") && config.find("portPrefix").isString())) {
        yInfo() << LogPrefix << "Using default portPrefix HDE ";
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(0.01)).asFloat64();
    pImpl->portPrefix = "/" + config.check("portPrefix", yarp::os::Value("HDE")).asString() + "/";
    pImpl->matioLogger = config.check("matioLogger", yarp::os::Value(false)).asBool();

    if (pImpl->matioLogger) {
        pImpl->matLogDirectory =  pImpl->originalWorkingDirectory.string() + "/" + config.check("matLogDirectory", yarp::os::Value("matLogDirectory")).asString() + "/";

        struct stat info;

        // check if matLogDirectory exists
        if (stat(pImpl->matLogDirectory.c_str(), &info) != 0 && mkdir(config.check("matLogDirectory", yarp::os::Value("matLogDirectory")).asString().c_str(), 0777) == -1) {
            yError() << LogPrefix << "Failed to created the matLogDirectory " << config.check("matLogDirectory", yarp::os::Value("matLogDirectory")).asString();
            return false;
        }

        std::experimental::filesystem::current_path(pImpl->matLogDirectory);
    }

    //TODO: Define rpc to reset the data dump or restarting the visualization??

    yInfo() << LogPrefix << "*** ===========================";
    yInfo() << LogPrefix << "*** Period                    :" << period;
    yInfo() << LogPrefix << "*** portPrefix                :" << pImpl->portPrefix;
    yInfo() << LogPrefix << "*** matioLogger               :" << pImpl->matioLogger;
    yInfo() << LogPrefix << "*** matLogDirectory           :" << pImpl->matLogDirectory;
    yInfo() << LogPrefix << "*** ===========================";

    // Set period
    setPeriod(pImpl->period);

    return true;
}

bool HumanDataCollector::close() {
    return true;
}

void HumanDataCollector::run()
{
    if (pImpl->firstData) {

        pImpl->firstData = false;

        // =====================================================================
        // Open yarp ports base on the attached devices for streaming human data
        // =====================================================================

        if (pImpl->isAttached.stateProvider) {

            // Open base pose data port
            const std::string basePosePortName = pImpl->portPrefix + DeviceName + "/basePose:o";
            if (!pImpl->basePoseDataPort.open(basePosePortName)) {
                yError() << LogPrefix << "Failed to open port " << basePosePortName;
                askToStop();
                return;
            }

            // Open base velocity data port
            const std::string baseVelocityPortName = pImpl->portPrefix + DeviceName + "/baseVelocity:o";
            if (!pImpl->baseVelocityDataPort.open(baseVelocityPortName)) {
                yError() << LogPrefix << "Failed to open port " << baseVelocityPortName;
                askToStop();
                return;
            }

            // Open state jonit names data port
            const std::string stateJointNamesPortName = pImpl->portPrefix + DeviceName + "/stateJointNames:o";
            if (!pImpl->stateJointNamesDataPort.open(stateJointNamesPortName)) {
                yError() << LogPrefix << "Failed to open port " << stateJointNamesPortName;
                askToStop();
                return;
            }

            // Open joint positions data port
            const std::string jointPositionsPortName = pImpl->portPrefix + DeviceName + "/jointPositions:o";
            if (!pImpl->jointPositionsDataPort.open(jointPositionsPortName)) {
                yError() << LogPrefix << "Failed to open port " << jointPositionsPortName;
                askToStop();
                return;
            }

            // Open joint velocities data port
            const std::string jointVelocitiesPortName = pImpl->portPrefix + DeviceName + "/jointVelocities:o";
            if (!pImpl->jointVelocitiesDataPort.open(jointVelocitiesPortName)) {
                yError() << LogPrefix << "Failed to open port " << jointVelocitiesPortName;
                askToStop();
                return;
            }

            // Open CoM position data port
            const std::string comPositionPortName = pImpl->portPrefix + DeviceName + "/comPosition:o";
            if (!pImpl->comPositionDataPort.open(comPositionPortName)) {
                yError() << LogPrefix << "Failed to open port " << comPositionPortName;
                askToStop();
                return;
            }

            // Open CoM velocity data port
            const std::string comVelocityPortName = pImpl->portPrefix + DeviceName + "/comVelocity:o";
            if (!pImpl->comVelocityDataPort.open(comVelocityPortName)) {
                yError() << LogPrefix << "Failed to open port " << comVelocityPortName;
                askToStop();
                return;
            }

            // Open CoM acceleration in base frame data port
            const std::string comProperAccelerationInBaseFramePortName = pImpl->portPrefix + DeviceName + "/comProperAccelerationInBaseFrame:o";
            if (!pImpl->comProperAccelerationInBaseFrameDataPort.open(comProperAccelerationInBaseFramePortName)) {
                yError() << LogPrefix << "Failed to open port " << comProperAccelerationInBaseFramePortName;
                askToStop();
                return;
            }

            // Open CoM acceleration in world frame data port
            const std::string comProperAccelerationInWorldFramePortName = pImpl->portPrefix + DeviceName + "/comProperAccelerationInWorldFrame:o";
            if (!pImpl->comProperAccelerationInWorldFrameDataPort.open(comProperAccelerationInWorldFramePortName)) {
                yError() << LogPrefix << "Failed to open port " << comProperAccelerationInWorldFramePortName;
                askToStop();
                return;
            }

        }

        if (pImpl->isAttached.wrenchProvider) {

            // Open wrench measurements source names data port
            const std::string wrenchMeasurementsSourceNamesPortName = pImpl->portPrefix + DeviceName + "/wrenchMeasurementSourceNames:o";
            if (!pImpl->wrenchMeasurementSourceNamesDataPort.open(wrenchMeasurementsSourceNamesPortName)) {
                yError() << LogPrefix << "Failed to open port " << wrenchMeasurementsSourceNamesPortName;
                askToStop();
                return;
            }

            // Open wrench measurement values data port
            const std::string wrenchMeausurementsDataPortName = pImpl->portPrefix + DeviceName + "/wrenchMeasurements:o";
            if (!pImpl->wrenchMeasurementValuesDataPort.open(wrenchMeausurementsDataPortName)) {
                yError() << LogPrefix << "Failed to open port " << wrenchMeausurementsDataPortName;
                askToStop();
                return;
            }

        }

        if (pImpl->isAttached.dynamicsEstimator) {

            // Open wrench estimates source names data port
            const std::string wrenchEstimatesSourceNamesPortName = pImpl->portPrefix + DeviceName + "/wrenchEstimateSourceNames:o";
            if (!pImpl->wrenchEstimateSourceNamesDataPort.open(wrenchEstimatesSourceNamesPortName)) {
                yError() << LogPrefix << "Failed to open port " << wrenchEstimatesSourceNamesPortName;
                askToStop();
                return;
            }

            // Open wrench estimate values data port
            const std::string wrenchEstimatesDataPortName = pImpl->portPrefix + DeviceName + "/wrenchEstimates:o";
            if (!pImpl->wrenchEstimateValuesDataPort.open(wrenchEstimatesDataPortName)) {
                yError() << LogPrefix << "Failed to open port " << wrenchEstimatesDataPortName;
                askToStop();
                return;
            }

            // Open dynamics joint names data port
            const std::string dynamicsJointNamesDataPortName = pImpl->portPrefix + DeviceName + "/dynamicsJointNames:o";
            if (!pImpl->dynamicsJointNamesDataPort.open(dynamicsJointNamesDataPortName)) {
                yError() << LogPrefix << "Failed to open port " << dynamicsJointNamesDataPortName;
                askToStop();
                return;
            }

            // Open joint torques data port
            const std::string jointTorquesDataPortName = pImpl->portPrefix + DeviceName + "/jointTorques:o";
            if (!pImpl->jointTorquesDataPort.open(jointTorquesDataPortName)) {
                yError() << LogPrefix << "Failed to open port " << jointTorquesDataPortName;
                askToStop();
                return;
            }

        }

        // Initialize  matio logging
        if (pImpl->matioLogger) {

            // Get the start time
            pImpl->startTime = std::chrono::system_clock::now();

            // TODO: Handle multiple files
            std::string matLogFileFullPathName = pImpl->matLogDirectory + pImpl->matLogFileName;
            pImpl->matFilePtr = Mat_CreateVer(matLogFileFullPathName.c_str(), nullptr, MAT_FT_MAT73);

            if (pImpl->matFilePtr == nullptr) {
                yError() << LogPrefix << "Failed to create " << pImpl->matLogFileName << " MAT file for logging";
                askToStop();
                return;
            }

            // Initialize time vector
            pImpl->humanDataStruct.time.clear();

            // Initialize human data struct buffers
            if (pImpl->isAttached.stateProvider) {

                pImpl->humanDataStruct.stateJointNames.clear();
                pImpl->humanDataStruct.data["basePose"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["baseVelocity"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.data["comPosition"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["comVelocity"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["comProperAccelerationInBaseFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["comProperAccelerationInWorldFrame"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.data["jointPositions"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["jointVelocities"] = std::vector<std::vector<double>>();
            }

            if (pImpl->isAttached.wrenchProvider) {

                pImpl->humanDataStruct.wrenchEstimateSourceNames.clear();
                pImpl->humanDataStruct.data["wrenchMeasurements"] = std::vector<std::vector<double>>();
            }

            if (pImpl->isAttached.dynamicsEstimator) {

                pImpl->humanDataStruct.wrenchEstimateSourceNames.clear();
                pImpl->humanDataStruct.data["wrenchEstimates"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.dynamicsJointNames.clear();
                pImpl->humanDataStruct.data["jointTorques"] = std::vector<std::vector<double>>();
            }

        }

    }


    // =========================================================
    // Get data from different interfaces from attached devices
    // =========================================================

    if (pImpl->matioLogger) {

        // Get the current time
        pImpl->currentTime = std::chrono::system_clock::now();

        // Get the duration from start time to current time
        std::chrono::duration<long, std::nano> timeDuration = pImpl->currentTime - pImpl->startTime;
        //yInfo() << LogPrefix << "Duration is " << timeDuration.count();

        // Push back time duration to a vector
        pImpl->humanDataStruct.time.push_back(timeDuration.count());

    }

    // Get data from IHumanState interface of HumanStateProvider
    if (pImpl->isAttached.stateProvider) {

        // Base Quantities
        pImpl->baseName = pImpl->iHumanState->getBaseName();
        pImpl->basePosition = pImpl->iHumanState->getBasePosition();
        pImpl->baseOrientation = pImpl->iHumanState->getBaseOrientation();
        pImpl->baseVelocity = pImpl->iHumanState->getBaseVelocity();

        // Joint Quantities
        pImpl->stateNumberOfJoints = pImpl->iHumanState->getNumberOfJoints();
        pImpl->stateJointNames = pImpl->iHumanState->getJointNames();
        pImpl->jointPositionsVec = pImpl->iHumanState->getJointPositions();
        pImpl->jointVelocitiesVec = pImpl->iHumanState->getJointVelocities();

        // CoM Quantities
        pImpl->comPosition = pImpl->iHumanState->getCoMPosition();
        pImpl->comVelocity = pImpl->iHumanState->getCoMVelocity();
        pImpl->comProperAccInBaseFrame = pImpl->iHumanState->getCoMProperAccelerationExpressedInBaseFrame();
        pImpl->comProperAccInWorldFrame = pImpl->iHumanState->getCoMProperAccelerationExpressedInWorldFrame();

    }

    // Get data from IHumanWrench interface of HumanWrenchProvider
    if (pImpl->isAttached.wrenchProvider) {

        pImpl->numberOfWrenchMeasurementSources = pImpl->iHumanWrenchMeasurements->getNumberOfWrenchSources();
        pImpl->wrenchMeasurementSourceNames = pImpl->iHumanWrenchMeasurements->getWrenchSourceNames();
        pImpl->wrenchMeasurementValuesVec = pImpl->iHumanWrenchMeasurements->getWrenches();

    }    

    if (pImpl->isAttached.dynamicsEstimator) {

        // Get data from IHumanWrench interface of HumanDynamicsEstimator
        // NOTE: The wrench values coming from HumanDynamicsEstimators are (offsetRemovedWrenchMeasurements & WrenchEstimates) of each link
        pImpl->numberOfWrenchEstimateSources = pImpl->iHumanWrenchEstimates->getNumberOfWrenchSources();
        pImpl->wrenchEstimateSourceNames = pImpl->iHumanWrenchEstimates->getWrenchSourceNames();
        pImpl->wrenchEstimateValuesVec = pImpl->iHumanWrenchEstimates->getWrenches();

        // Get data from IHumanDynamics interface of HumanDynamicsEstimator
        pImpl->dynamicsNumberOfJoints = pImpl->iHumanDynamics->getNumberOfJoints();
        pImpl->dynamicsJointNames = pImpl->iHumanDynamics->getJointNames();
        pImpl->jointTorquesVec = pImpl->iHumanDynamics->getJointTorques();

    }

    // Prepare buffer vectors
    // Get base pose array from base position and orientation
    if (pImpl->isAttached.stateProvider) {

        pImpl->basePoseArray[0] = pImpl->basePosition[0];
        pImpl->basePoseArray[1] = pImpl->basePosition[1];
        pImpl->basePoseArray[2] = pImpl->basePosition[2];
        pImpl->basePoseArray[3] = pImpl->baseOrientation[0];
        pImpl->basePoseArray[4] = pImpl->baseOrientation[1];
        pImpl->basePoseArray[5] = pImpl->baseOrientation[2];
        pImpl->basePoseArray[6] = pImpl->baseOrientation[3];

        pImpl->basePoseVec = std::vector<double>(pImpl->basePoseArray.begin(), pImpl->basePoseArray.end());

        pImpl->baseVeclocityVec = std::vector<double>(pImpl->baseVelocity.begin(), pImpl->baseVelocity.end());

        pImpl->comPositionVec = std::vector<double>(pImpl->comPosition.begin(), pImpl->comPosition.end());

        pImpl->comVelocityVec = std::vector<double>(pImpl->comVelocity.begin(), pImpl->comVelocity.end());

        pImpl->comProperAccInBaseFrameVec = std::vector<double>(pImpl->comProperAccInBaseFrame.begin(), pImpl->comProperAccInBaseFrame.end());

        pImpl->comProperAccInWorldFrameVec = std::vector<double>(pImpl->comProperAccInWorldFrame.begin(), pImpl->comProperAccInWorldFrame.end());

    }

    // Update interface data to matio buffers
    if (pImpl->matioLogger) {

        if (pImpl->isAttached.stateProvider) {

            // Set state joint names once
            if (pImpl->humanDataStruct.stateJointNames.empty()) {
                pImpl->humanDataStruct.stateJointNames = pImpl->stateJointNames;
            }


            pImpl->humanDataStruct.data.at("basePose").push_back(pImpl->basePoseVec);
            pImpl->humanDataStruct.data.at("baseVelocity").push_back(pImpl->baseVeclocityVec);

            pImpl->humanDataStruct.data.at("comPosition").push_back(pImpl->comPositionVec);
            pImpl->humanDataStruct.data.at("comVelocity").push_back(pImpl->comVelocityVec);
            pImpl->humanDataStruct.data.at("comProperAccelerationInBaseFrame").push_back(pImpl->comProperAccInBaseFrameVec);
            pImpl->humanDataStruct.data.at("comProperAccelerationInWorldFrame").push_back(pImpl->comProperAccInWorldFrameVec);

            pImpl->humanDataStruct.data.at("jointPositions").push_back(pImpl->jointPositionsVec);
            pImpl->humanDataStruct.data.at("jointVelocities").push_back(pImpl->jointVelocitiesVec);


        }

        if (pImpl->isAttached.wrenchProvider) {

            // Set wrench measurements source names once
            if (pImpl->humanDataStruct.wrenchMeasurementSourceNames.empty()) {
                pImpl->humanDataStruct.wrenchMeasurementSourceNames = pImpl->wrenchMeasurementSourceNames;
            }

            pImpl->humanDataStruct.data["wrenchMeasurements"].push_back(pImpl->wrenchMeasurementValuesVec);

        }

        if (pImpl->isAttached.dynamicsEstimator) {

            // Set wrench estimates source names once
            if (pImpl->humanDataStruct.wrenchEstimateSourceNames.empty()) {
                pImpl->humanDataStruct.wrenchEstimateSourceNames = pImpl->wrenchEstimateSourceNames;
            }

            pImpl->humanDataStruct.data["wrenchEstimates"].push_back(pImpl->wrenchEstimateValuesVec);

            // Set dynamics joint names once
            if (pImpl->humanDataStruct.dynamicsJointNames.empty()) {
                pImpl->humanDataStruct.dynamicsJointNames = pImpl->dynamicsJointNames;
            }

            pImpl->humanDataStruct.data["jointTorques"].push_back(pImpl->jointTorquesVec);
        }

    }

    // ============================
    // Put human data to yarp ports
    // ============================

    if (pImpl->isAttached.stateProvider) {

        // Prepare base pose data
        yarp::sig::Vector& basePoseYarpVector = pImpl->basePoseDataPort.prepare();
        YarpConversionsHelper::toYarp(basePoseYarpVector, pImpl->basePoseVec);

        // Prepare base velocity data
        yarp::sig::Vector& baseVelocityYarpVector = pImpl->baseVelocityDataPort.prepare();
        YarpConversionsHelper::toYarp(baseVelocityYarpVector, pImpl->baseVeclocityVec);

        // Prepare stateJointNames
        yarp::os::Bottle& stateJointNamesYarpBottle = pImpl->stateJointNamesDataPort.prepare();
        YarpConversionsHelper::toYarp(stateJointNamesYarpBottle, pImpl->stateJointNames);

        // Prepare joint positions
        yarp::sig::Vector& jointPositionsYarpVector = pImpl->jointPositionsDataPort.prepare();
        YarpConversionsHelper::toYarp(jointPositionsYarpVector, pImpl->jointPositionsVec);

        // Prepare joint velocities
        yarp::sig::Vector& jointVelocitiesYarpVector = pImpl->jointVelocitiesDataPort.prepare();
        YarpConversionsHelper::toYarp(jointVelocitiesYarpVector, pImpl->jointVelocitiesVec);

        // Preprare com position
        yarp::sig::Vector& comPositionYarpVector = pImpl->comPositionDataPort.prepare();
        YarpConversionsHelper::toYarp(comPositionYarpVector, pImpl->comPositionVec);

        // Preprate com velocity
        yarp::sig::Vector& comVelocityYarpVector = pImpl->comVelocityDataPort.prepare();
        YarpConversionsHelper::toYarp(comVelocityYarpVector, pImpl->comVelocityVec);

        // Prepare com proper acceleration in base frame
        yarp::sig::Vector& comProperAccelerationInBaseFrameYarpVector = pImpl->comProperAccelerationInBaseFrameDataPort.prepare();
        YarpConversionsHelper::toYarp(comProperAccelerationInBaseFrameYarpVector, pImpl->comProperAccInBaseFrameVec);


        // Prepare com proper acceleration in world frame
        yarp::sig::Vector& comProperAccelerationInWorldFrameYarpVector = pImpl->comProperAccelerationInWorldFrameDataPort.prepare();
        YarpConversionsHelper::toYarp(comProperAccelerationInWorldFrameYarpVector, pImpl->comProperAccInWorldFrameVec);

        // Send data through yarp ports
        pImpl->basePoseDataPort.write(true);
        pImpl->baseVelocityDataPort.write(true);
        pImpl->stateJointNamesDataPort.write(true);
        pImpl->jointPositionsDataPort.write(true);
        pImpl->jointVelocitiesDataPort.write(true);
        pImpl->comPositionDataPort.write(true);
        pImpl->comVelocityDataPort.write(true);
        pImpl->comProperAccelerationInBaseFrameDataPort.write(true);
        pImpl->comProperAccelerationInWorldFrameDataPort.write(true);
    }

    if (pImpl->isAttached.wrenchProvider) {

        // Prepare wrench measurements source names data
        yarp::os::Bottle& wrenchMeasurementSourceNamesYarpBottle = pImpl->wrenchMeasurementSourceNamesDataPort.prepare();
        YarpConversionsHelper::toYarp(wrenchMeasurementSourceNamesYarpBottle, pImpl->wrenchMeasurementSourceNames);

        // Prepare wrench measurement values data
        yarp::sig::Vector& wrenchMeasurementValuesYarpVector = pImpl->wrenchMeasurementValuesDataPort.prepare();
        YarpConversionsHelper::toYarp(wrenchMeasurementValuesYarpVector, pImpl->wrenchMeasurementValuesVec);

        // Send data through yarp ports
        pImpl->wrenchMeasurementSourceNamesDataPort.write(true);
        pImpl->wrenchMeasurementValuesDataPort.write(true);
    }

    if (pImpl->isAttached.dynamicsEstimator) {

        // Prepare wrench estimates source names data
        yarp::os::Bottle& wrenchEstimateSourceNamesYarpBottle = pImpl->wrenchEstimateSourceNamesDataPort.prepare();
        YarpConversionsHelper::toYarp(wrenchEstimateSourceNamesYarpBottle, pImpl->wrenchEstimateSourceNames);

        // Prepare wrench estimate values data
        yarp::sig::Vector& wrenchEstimateValuesYarpVector = pImpl->wrenchEstimateValuesDataPort.prepare();
        YarpConversionsHelper::toYarp(wrenchEstimateValuesYarpVector, pImpl->wrenchEstimateValuesVec);

        // Prepare dynamics joint names data
        yarp::os::Bottle& dynamicsJointNamesYarpBottle = pImpl->dynamicsJointNamesDataPort.prepare();
        YarpConversionsHelper::toYarp(dynamicsJointNamesYarpBottle, pImpl->dynamicsJointNames);

        // Prepare joint torques data
        yarp::sig::Vector& jointTorqesYarpVector = pImpl->jointTorquesDataPort.prepare();
        YarpConversionsHelper::toYarp(jointTorqesYarpVector, pImpl->jointTorquesVec);

        // Send data through yarp ports
        pImpl->wrenchEstimateSourceNamesDataPort.write(true);
        pImpl->wrenchEstimateValuesDataPort.write(true);
        pImpl->dynamicsJointNamesDataPort.write(true);
        pImpl->jointTorquesDataPort.write(true);
    }
}

bool HumanDataCollector::attach(yarp::dev::PolyDriver *poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver device is nullptr";
        return false;
    }

    // Check the polydriver options for device option
    yarp::os::Bottle driverOptions = poly->getOptions();
    if (!driverOptions.check("device")) {
        yError() << LogPrefix << "Passed polydriver does not have <device> as an option";
        return false;
    }

    // Get the passed device name
    const std::string deviceName = poly->getValue("device").asString();

    // Ensure the passed device is acceptable
    if (deviceName != "human_state_provider" && deviceName != "human_wrench_provider" && deviceName != "human_dynamics_estimator") {
        yError() << "This wrapper does not accept the " << deviceName << " passed as the polydriver device for attaching";
        return  false;
    }

    if (deviceName == "human_state_provider") {

        // Attach to IHumanState interface from HumanStateProvider
        if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
            yError() << LogPrefix << "Failed to view IHumanState interface from the " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanState->getNumberOfJoints() == 0 || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
            yError() << "The viewed IHumanState interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
        pImpl->isAttached.stateProvider = true;
    }

    if (deviceName == "human_wrench_provider") {

        // Attach to IHumanWrench interface from HumanWrenchProvider
        if (pImpl->iHumanWrenchMeasurements || !poly->view(pImpl->iHumanWrenchMeasurements) || !pImpl->iHumanWrenchMeasurements) {
            yError() << LogPrefix << "Failed to view IHumanWrench interface from the " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanWrenchMeasurements->getNumberOfWrenchSources() != pImpl->iHumanWrenchMeasurements->getWrenchSourceNames().size()) {
            yError() << "The viewed IHumanWrench interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
        pImpl->isAttached.wrenchProvider = true;

    }

    if (deviceName == "human_dynamics_estimator") {

        // Attach to IHumanWrench interface from HumanDynamicsEstimator
        if (pImpl->iHumanWrenchEstimates || !poly->view(pImpl->iHumanWrenchEstimates) || !pImpl->iHumanWrenchEstimates) {
            yError() << LogPrefix << "Failed to view IHumanWrench interface from the " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanWrenchEstimates->getNumberOfWrenchSources() != pImpl->iHumanWrenchEstimates->getWrenchSourceNames().size()) {
            yError() << "The viewed IHumanWrench interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        // Attach to IHumanDynamics interface from HumanDynamicsEstimator
        if (pImpl->iHumanDynamics || !poly->view(pImpl->iHumanDynamics) || !pImpl->iHumanDynamics) {
            yError() << LogPrefix << "Failed to view IHumanDynamics interface from the  " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanDynamics->getNumberOfJoints() == 0 || pImpl->iHumanDynamics->getNumberOfJoints() != pImpl->iHumanDynamics->getJointNames().size()) {
            yError() << "The viewed IHumanDynamics interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
        pImpl->isAttached.dynamicsEstimator = true;
    }

    return true;
}

bool HumanDataCollector::detach()
{
    while (isRunning()) {
        stop();
    }

    if (pImpl->matioLogger) {

        // Initialize the mat struct
        size_t matDataStructDims[2] = {1,1};

        // Set the mat struct field names

        // Handle individual filed names
        std::vector<std::string> matStructFieldNamesVec;
        matStructFieldNamesVec.push_back("time");
        matStructFieldNamesVec.push_back("stateJointNames");
        matStructFieldNamesVec.push_back("wrenchMeasurementSourceNames");
        matStructFieldNamesVec.push_back("wrenchEstimateSourceNames");
        matStructFieldNamesVec.push_back("dynamicsJointNames");

        for (std::unordered_map<std::string, std::vector<std::vector<double>>>::iterator it = pImpl->humanDataStruct.data.begin(); it != pImpl->humanDataStruct.data.end(); it++) {
            matStructFieldNamesVec.push_back(it->first);
        }

        // Construct vector of character pointers to field names
        std::vector<const char*> fieldNames;
        for (size_t i = 0; i < matStructFieldNamesVec.size(); i++) {
            fieldNames.push_back(matStructFieldNamesVec[i].c_str());
        }

        // Initialize mat struct variabel
        pImpl->matDataStruct = Mat_VarCreateStruct("data", 1, matDataStructDims, fieldNames.data(), matStructFieldNamesVec.size());

        if (pImpl->matDataStruct == nullptr) {
            yError() << LogPrefix << "Failed to initialize matio struct variable";
            Mat_VarFree(pImpl->matDataStruct);
            Mat_Close(pImpl->matFilePtr);
            return false;
        }

        // Correct the time values stored in the time vector to zero start value
        std::transform( pImpl->humanDataStruct.time.begin(), pImpl->humanDataStruct.time.end(), pImpl->humanDataStruct.time.begin(), std::bind2nd( std::plus<long>(), -pImpl->humanDataStruct.time.at(0) ) );

        yInfo() << LogPrefix << "Duration vector size is " << pImpl->humanDataStruct.time.size() << " first element is " << pImpl->humanDataStruct.time.at(0);

        long time[pImpl->humanDataStruct.time.size()];
        std::copy(pImpl->humanDataStruct.time.begin(), pImpl->humanDataStruct.time.end(), time);

        yInfo() << LogPrefix << "Array size is " << sizeof(time)/sizeof(time[0]) << " first element of the array is " << time[0];

        size_t dims[2] {sizeof(time)/sizeof(time[0]),1};
        matvar_t *matTime = Mat_VarCreate(fieldNames[0], MAT_C_UINT64, MAT_T_UINT64, 2, dims, time, 0);
        matvar_t *time2 = Mat_VarCreate(fieldNames[1], MAT_C_UINT64, MAT_T_UINT64, 2, dims, time, 0);

        if (matTime == nullptr) {
            yError() << LogPrefix << "Failed to created a numeric cell array of time variable";
            Mat_VarFree(matTime);
            Mat_Close(pImpl->matFilePtr);
            return false;
        }

        // Pad mat struct with data
        Mat_VarSetStructFieldByName(pImpl->matDataStruct, fieldNames[0], 0, matTime);
        Mat_VarSetStructFieldByName(pImpl->matDataStruct, fieldNames[1], 0, time2);


        // Write mat struct to mat file before closing
        Mat_VarWrite(pImpl->matFilePtr, pImpl->matDataStruct, MAT_COMPRESSION_NONE);
        Mat_VarFree(pImpl->matDataStruct);


        // Close the mat file
        Mat_Close(pImpl->matFilePtr);

    }

    // Moving back a directory above the matLogDirectory
    std::experimental::filesystem::current_path(pImpl->originalWorkingDirectory);

    if (pImpl->isAttached.stateProvider) {

        if (!pImpl->basePoseDataPort.isClosed()) {
            pImpl->basePoseDataPort.close();
        }

        if (!pImpl->baseVelocityDataPort.isClosed()) {
            pImpl->baseVelocityDataPort.close();
        }

        if (!pImpl->stateJointNamesDataPort.isClosed()) {
            pImpl->stateJointNamesDataPort.close();
        }

        if (!pImpl->jointPositionsDataPort.isClosed()) {
            pImpl->jointPositionsDataPort.close();
        }

        if (!pImpl->jointVelocitiesDataPort.isClosed()) {
            pImpl->jointVelocitiesDataPort.close();
        }

        if (!pImpl->comPositionDataPort.isClosed()) {
            pImpl->comPositionDataPort.close();
        }

        if (!pImpl->comVelocityDataPort.isClosed()) {
            pImpl->comVelocityDataPort.close();
        }

        if (!pImpl->comProperAccelerationInBaseFrameDataPort.isClosed()) {
            pImpl->comProperAccelerationInBaseFrameDataPort.close();
        }

        if (!pImpl->comProperAccelerationInWorldFrameDataPort.isClosed()) {
            pImpl->comProperAccelerationInWorldFrameDataPort.close();
        }

        pImpl->iHumanState = nullptr;
        pImpl->isAttached.stateProvider = false;

    }

    if (pImpl->isAttached.wrenchProvider) {

        if (!pImpl->wrenchMeasurementSourceNamesDataPort.isClosed()) {
            pImpl->wrenchMeasurementSourceNamesDataPort.close();
        }

        if (!pImpl->wrenchMeasurementValuesDataPort.isClosed()) {
            pImpl->wrenchMeasurementValuesDataPort.close();
        }

        pImpl->iHumanWrenchMeasurements = nullptr;
        pImpl->isAttached.wrenchProvider = false;
    }

    if (pImpl->isAttached.dynamicsEstimator) {

        if (!pImpl->wrenchEstimateSourceNamesDataPort.isClosed()) {
            pImpl->wrenchEstimateSourceNamesDataPort.close();
        }

        if (!pImpl->wrenchEstimateValuesDataPort.isClosed()) {
            pImpl->wrenchEstimateValuesDataPort.close();
        }

        if (!pImpl->dynamicsJointNamesDataPort.isClosed()) {
            pImpl->dynamicsJointNamesDataPort.close();
        }

        if (!pImpl->jointTorquesDataPort.isClosed()) {
            pImpl->jointTorquesDataPort.close();
        }

        pImpl->iHumanWrenchEstimates = nullptr;
        pImpl->iHumanDynamics = nullptr;
        pImpl->isAttached.dynamicsEstimator = false;
    }

    return true;
}

bool HumanDataCollector::attachAll(const yarp::dev::PolyDriverList &driverList)
{
    bool attachStatus = true;

    if (driverList.size() > 3) {
        yError() << LogPrefix << "This wrapper accepts at most three attached PolyDriver devices";
        return false;
    }

    for (size_t i = 0; i < driverList.size(); i++) {
        const yarp::dev::PolyDriverDescriptor* driver = driverList[i];

        if (!driver) {
            yError() << LogPrefix << "Passed PolyDriver device is nullptr";
            return false;
        }

        attachStatus = attachStatus && attach(driver->poly);
    }

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!(attachStatus && start())) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    return attachStatus;
}

bool HumanDataCollector::detachAll()
{
    return detach();
}
