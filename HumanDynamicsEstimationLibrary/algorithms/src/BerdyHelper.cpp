/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "hde/algorithms/BerdyHelper.hpp"

#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/SparseMatrix.h>

#include <iDynTree/Model/Dynamics.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Sensors/AllSensorsTypes.h>
#include <iDynTree/Sensors/Sensors.h>

#include <iDynTree/Estimation/ExternalWrenchesEstimation.h>

#include <algorithm>
#include <sstream>

using namespace hde::algorithms;

bool isJointBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType)
{
    switch (dynamicVariableType) {
        case LINK_BODY_PROPER_ACCELERATION:
            return false;
        case NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            return false;
        case JOINT_WRENCH:
            return true;
        case DOF_TORQUE:
            return false;
        case NET_EXT_WRENCH:
            return false;
        case DOF_ACCELERATION:
            return false;
        case LINK_BODY_PROPER_CLASSICAL_ACCELERATION:
            return false;
        default:
            return false;
    }
}

bool isLinkBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType)
{
    switch (dynamicVariableType) {
        case LINK_BODY_PROPER_ACCELERATION:
            return true;
        case NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            return true;
        case JOINT_WRENCH:
            return false;
        case DOF_TORQUE:
            return false;
        case NET_EXT_WRENCH:
            return true;
        case DOF_ACCELERATION:
            return false;
        case LINK_BODY_PROPER_CLASSICAL_ACCELERATION:
            return true;
        default:
            return false;
    }
}

bool isDOFBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType)
{
    switch (dynamicVariableType) {
        case LINK_BODY_PROPER_ACCELERATION:
            return false;
        case NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            return false;
        case JOINT_WRENCH:
            return false;
        case DOF_TORQUE:
            return true;
        case NET_EXT_WRENCH:
            return false;
        case DOF_ACCELERATION:
            return true;
        case LINK_BODY_PROPER_CLASSICAL_ACCELERATION:
            return false;
        default:
            return false;
    }
}

bool BerdyOptions::checkConsistency()
{
    if (this->includeAllNetExternalWrenchesAsSensors) {
        if (!this->includeAllNetExternalWrenchesAsDynamicVariables) {
            iDynTree::reportError(
                "BerdyOptions",
                "checkConsistency",
                "Impossible to load berdy, as includeAllNetExternalWrenchesAsSensors is set to "
                "true but includeAllNetExternalWrenchesAsDynamicVariables is set to false");
            return false;
        }
    }

    return true;
}

bool BerdySensor::operator==(const struct BerdySensor& s) const
{
    return s.type == this->type && s.id == this->id;
}

bool BerdySensor::operator<(const struct BerdySensor& s) const
{
    return this->range.offset < s.range.offset;
}

bool BerdyDynamicVariable::operator==(const struct BerdyDynamicVariable& v) const
{
    return v.type == this->type && v.id == this->id;
}

bool BerdyDynamicVariable::operator<(const struct BerdyDynamicVariable& v) const
{
    return this->range.offset < v.range.offset;
}

BerdyHelper::BerdyHelper()
    : m_areModelAndSensorsValid(false)
    , m_kinematicsUpdated(false)
    , m_nrOfDynamicalVariables(0)
    , m_nrOfDynamicEquations(0)
    , m_nrOfSensorsMeasurements(0)
{}

bool BerdyHelper::init(const iDynTree::Model& model,
                       const iDynTree::SensorsList& sensors,
                       const BerdyOptions options)
{
    // Reset the class
    m_kinematicsUpdated = false;
    m_areModelAndSensorsValid = false;

    m_model = model;
    m_sensors = sensors;
    m_options = options;

    iDynTree::LinkIndex baseLinkIndex;
    if (!options.baseLink.empty()) {
        // find the LinkIndex corresponding to the baseLink option
        baseLinkIndex = m_model.getLinkIndex(options.baseLink);

        if (baseLinkIndex == iDynTree::LINK_INVALID_INDEX) {
            iDynTree::reportError(
                "BerdyHelpers",
                "init",
                ("Error while setting base link  to " + options.baseLink + ".").c_str());
            return false;
        }
    }
    else {
        baseLinkIndex = m_model.getDefaultBaseLink();
    }

    m_model.computeFullTreeTraversal(m_dynamicsTraversal, baseLinkIndex);
    m_kinematicTraversals.resize(m_model);
    m_jointPos.resize(m_model);
    m_jointVel.resize(m_model);
    m_linkVels.resize(m_model);
    m_link_H_externalWrenchMeasurementFrame.resize(m_model.getNrOfLinks(),
                                                   iDynTree::Transform::Identity());

    bool res = m_options.checkConsistency();

    if (!res) {
        iDynTree::reportError("BerdyHelpers", "init", "BerdyOptions not consistent.");
        return false;
    }

    switch (m_options.berdyVariant) {
        case ORIGINAL_BERDY_FIXED_BASE:
            res = initOriginalBerdyFixedBase();
            cacheDynamicVariablesOrderingFixedBase();
            break;

        case BERDY_FLOATING_BASE:
            res = initBerdyFloatingBase();
            cacheDynamicVariablesOrderingFloatingBase();
            break;

        default:
            iDynTree::reportError("BerdyHelpers", "init", "unknown berdy variant");
            res = false;
    }

    if (res) {
        m_areModelAndSensorsValid = true;
    }
    else {
        iDynTree::reportError("BerdyHelpers", "init", "initialization failed");
    }

    return res;
}

BerdyOptions BerdyHelper::getOptions() const
{
    return m_options;
}

bool BerdyHelper::initSensorsMeasurements()
{
    // The number of sensors measurements is given by the sensors contained
    // in sensorsList (informally: the number of measurments of sensors
    // contained in the robot model) plus the additional/fictious sensors
    // specified in the BerdyOptions
    m_nrOfSensorsMeasurements = m_sensors.getSizeOfAllSensorsMeasurements();

    // The offset of joint accelerations
    berdySensorTypeOffsets.dofAccelerationOffset = m_nrOfSensorsMeasurements;

    if (m_options.includeAllJointAccelerationsAsSensors) {
        m_nrOfSensorsMeasurements += this->m_model.getNrOfDOFs();
    }

    // The offset of joint torques
    berdySensorTypeOffsets.dofTorquesOffset = m_nrOfSensorsMeasurements;

    if (m_options.includeAllJointTorquesAsSensors) {
        m_nrOfSensorsMeasurements += this->m_model.getNrOfDOFs();
    }

    berdySensorTypeOffsets.netExtWrenchOffset = m_nrOfSensorsMeasurements;
    if (m_options.includeAllNetExternalWrenchesAsSensors) {
        unsigned numOfExternalWrenches = this->m_model.getNrOfLinks();
        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE
            && !m_options.includeFixedBaseExternalWrench)
            numOfExternalWrenches = this->m_model.getNrOfLinks() - 1;
        m_nrOfSensorsMeasurements += 6 * numOfExternalWrenches;
    }

    berdySensorTypeOffsets.jointWrenchOffset = m_nrOfSensorsMeasurements;

    // Check the considered joint wrenches are actually part of the model
    berdySensorsInfo.jntIdxToOffset.resize(m_model.getNrOfJoints(), iDynTree::JOINT_INVALID_INDEX);
    berdySensorsInfo.wrenchSensors.clear();
    berdySensorsInfo.wrenchSensors.reserve(
        m_options.jointOnWhichTheInternalWrenchIsMeasured.size());
    for (size_t i = 0; i < m_options.jointOnWhichTheInternalWrenchIsMeasured.size(); i++) {
        iDynTree::JointIndex jntIdx =
            m_model.getJointIndex(m_options.jointOnWhichTheInternalWrenchIsMeasured[i]);

        if (jntIdx == iDynTree::JOINT_INVALID_INDEX) {
            std::stringstream ss;
            ss << "unknown joint " << m_options.jointOnWhichTheInternalWrenchIsMeasured[i];
            iDynTree::reportError("BerdyHelper", "initSensorsMeasurements", ss.str().c_str());
            return false;
        }

        berdySensorsInfo.wrenchSensors.push_back(jntIdx);
        berdySensorsInfo.jntIdxToOffset[jntIdx] = m_nrOfSensorsMeasurements;
        m_nrOfSensorsMeasurements += 6;
    }

    // Create sensor ordering vector
    cacheSensorsOrdering();

    return true;
}

bool BerdyHelper::initOriginalBerdyFixedBase()
{
    // Check that all joints have 1 dof
    for (iDynTree::JointIndex jntIdx = 0;
         jntIdx < static_cast<iDynTree::JointIndex>(m_model.getNrOfJoints());
         jntIdx++) {
        if (m_model.getJoint(jntIdx)->getNrOfDOFs() != 1) {
            std::stringstream ss;
            ss << "Joint " << m_model.getJointName(jntIdx) << " has "
               << m_model.getJoint(jntIdx)->getNrOfDOFs()
               << " DOFs , but the original fixed base formulation of berdy only supports 1 dof "
                  "frames";
            iDynTree::reportError("BerdyHelpers", "init", ss.str().c_str());
            return false;
        }
    }

    // Check that no sensors are attached to the base (this is not supported by the original berdy)
    for (iDynTree::SensorType type = iDynTree::SIX_AXIS_FORCE_TORQUE;
         type < iDynTree::NR_OF_SENSOR_TYPES;
         type = (iDynTree::SensorType) (type + 1)) {
        // TODO : link sensor are extremly widespared and they have all approximatly the same API,
        //        so we need a better way to iterate over them
        if (isLinkSensor(type)) {
            for (size_t sensIdx = 0; sensIdx < m_sensors.getNrOfSensors(type); sensIdx++) {
                iDynTree::LinkSensor* linkSensor =
                    dynamic_cast<iDynTree::LinkSensor*>(m_sensors.getSensor(type, sensIdx));

                iDynTree::LinkIndex linkToWhichTheSensorIsAttached =
                    linkSensor->getParentLinkIndex();

                if (linkToWhichTheSensorIsAttached
                    == m_dynamicsTraversal.getBaseLink()->getIndex()) {
                    std::stringstream ss;
                    ss << "Sensor " << linkSensor->getName() << " is attached to link "
                       << m_model.getLinkName(linkToWhichTheSensorIsAttached)
                       << " but this link is the base link and base link sensors are not supported "
                          "by the original berdy.";
                    iDynTree::reportError("BerdyHelpers", "init", ss.str().c_str());
                    return false;
                }
            }
        }
    }

    // N will be the number of joints/links/dofs
    // We assume that the fixed base is fixed, so we include in the model only N links
    size_t nrOfDOFs = m_model.getNrOfDOFs();

    // In the classical Berdy formulation, we have 6*4 + 2*1 dynamical variables
    size_t nrOfDynamicVariablesForDOFs =
        m_options.includeAllNetExternalWrenchesAsDynamicVariables ? 6 * 4 + 2 * 1 : 6 * 3 + 2 * 1;
    m_nrOfDynamicalVariables = nrOfDynamicVariablesForDOFs * nrOfDOFs;
    // 19 Dynamics equations are considered in the original berdy :
    // 6 for the propagation of 6d acceleration
    // 6 for the computation of the net wrench from acc and vel
    // 6 for the computation of the joint wrench
    // 1 for the torque computations
    m_nrOfDynamicEquations = (18 + 1) * nrOfDOFs;

    initSensorsMeasurements();

    return true;
}

iDynTree::IndexRange
BerdyHelper::getRangeOriginalBerdyFixedBase(BerdyDynamicVariablesTypes dynamicVariableType,
                                            iDynTree::TraversalIndex idx) const
{
    iDynTree::IndexRange ret;
    size_t dynamicVariableSize, dynamicVariableOffset, nrOfVariablesForJoint;

    if (m_options.includeAllNetExternalWrenchesAsDynamicVariables) {
        nrOfVariablesForJoint = 26;
    }
    else {
        nrOfVariablesForJoint = 20;
    }

    switch (dynamicVariableType) {
        case LINK_BODY_PROPER_ACCELERATION:
            dynamicVariableSize = 6;
            dynamicVariableOffset = 0;
            break;
        case NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            dynamicVariableSize = 6;
            dynamicVariableOffset = 6;
            break;
        case JOINT_WRENCH:
            dynamicVariableSize = 6;
            dynamicVariableOffset = 12;
            break;
        case DOF_TORQUE:
            dynamicVariableSize = 1;
            dynamicVariableOffset = 18;
            break;
        case NET_EXT_WRENCH:
            dynamicVariableSize = m_options.includeAllNetExternalWrenchesAsDynamicVariables ? 6 : 0;
            dynamicVariableOffset = 19;
            break;
        case DOF_ACCELERATION:
            dynamicVariableSize = 1;
            dynamicVariableOffset =
                m_options.includeAllNetExternalWrenchesAsDynamicVariables ? 25 : 19;
            break;
        default:
            assert(false);
            dynamicVariableSize = 0;
            dynamicVariableOffset = 0;
            break;
    }

    // Note that the traversal index 0 is for the base link, that is not considered
    // in the fixed base Berdy, so we start from the link with traversal index 1
    ret.offset = nrOfVariablesForJoint * (idx - 1) + dynamicVariableOffset;
    ret.size = dynamicVariableSize;

    return ret;
}

iDynTree::IndexRange
BerdyHelper::getRangeLinkVariable(BerdyDynamicVariablesTypes dynamicVariableType,
                                  iDynTree::LinkIndex idx) const
{
    if( !isLinkBerdyDynamicVariable(dynamicVariableType) )
    {
        return iDynTree::IndexRange::InvalidRange();
    }

    if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        return getRangeOriginalBerdyFixedBase(
            dynamicVariableType, m_dynamicsTraversal.getTraversalIndexFromLinkIndex(idx));
    }
    else {
        iDynTree::IndexRange ret;
        assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
        assert(m_options.includeAllNetExternalWrenchesAsDynamicVariables);
        // For BERDY_FLOATING_BASE, the only two link dynamic variable are the proper classical
        // acceleration and the external force-torque
        switch (dynamicVariableType) {
            case LINK_BODY_PROPER_CLASSICAL_ACCELERATION:
                ret.offset = 12 * idx;
                ret.size = 6;
                break;
            case NET_EXT_WRENCH:
                ret.offset = 12 * idx + 6;
                ret.size = 6;
                break;
            default:
                assert(false);
                ret = iDynTree::IndexRange::InvalidRange();
                break;
        }

        return ret;
    }
}

// \todo TODO remove
iDynTree::TraversalIndex getTraversalIndexFromJointIndex(const iDynTree::Model& m_model,
                                                         const iDynTree::Traversal& m_traversal,
                                                         const iDynTree::JointIndex idx)
{
    iDynTree::LinkIndex childLink = m_traversal.getChildLinkIndexFromJointIndex(m_model, idx);

    return m_traversal.getTraversalIndexFromLinkIndex(childLink);
}

iDynTree::IndexRange
BerdyHelper::getRangeJointVariable(BerdyDynamicVariablesTypes dynamicVariableType,
                                   iDynTree::JointIndex idx) const
{
    if( !isJointBerdyDynamicVariable(dynamicVariableType) )
    {
        assert(false);
        return iDynTree::IndexRange::InvalidRange();
    }

    if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        return getRangeOriginalBerdyFixedBase(
            dynamicVariableType,
            getTraversalIndexFromJointIndex(m_model, m_dynamicsTraversal, idx));
    }
    else {
        iDynTree::IndexRange ret;
        assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
        assert(m_options.includeAllNetExternalWrenchesAsDynamicVariables);

        int totalSizeOfLinkVariables = 12 * m_model.getNrOfLinks();

        switch (dynamicVariableType) {
            case JOINT_WRENCH:
                ret.offset = totalSizeOfLinkVariables + 6 * idx;
                ret.size = 6;
                break;
            default:
                assert(false);
                ret = iDynTree::IndexRange::InvalidRange();
                break;
        }

        return ret;
    }
}

iDynTree::IndexRange
BerdyHelper::getRangeDOFVariable(BerdyDynamicVariablesTypes dynamicVariableType,
                                 iDynTree::DOFIndex idx) const
{
    if( !isDOFBerdyDynamicVariable(dynamicVariableType) )
    {
        return iDynTree::IndexRange::InvalidRange();
    }

    if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        // For a model with all 1-dofs, dof index and joint index matches
        return getRangeOriginalBerdyFixedBase(
            dynamicVariableType,
            getTraversalIndexFromJointIndex(m_model, m_dynamicsTraversal, idx));
    }
    else {
        iDynTree::IndexRange ret;
        assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
        assert(m_options.includeAllNetExternalWrenchesAsDynamicVariables);

        int totalSizeOfLinkVariables = 12 * m_model.getNrOfLinks();
        int totalSizeOfJointVariables = 6 * m_model.getNrOfJoints();

        switch (dynamicVariableType) {
            case DOF_ACCELERATION:
                ret.offset = totalSizeOfLinkVariables + totalSizeOfJointVariables + idx;
                ret.size = 1;
                break;
            default:
                assert(false);
                ret = iDynTree::IndexRange::InvalidRange();
                break;
        }

        return ret;
    }
}

iDynTree::IndexRange BerdyHelper::getRangeSensorVariable(const iDynTree::SensorType type,
                                                         const unsigned int sensorIdx) const
{
    unsigned int sensorOffset = 0;

    if (type > iDynTree::SIX_AXIS_FORCE_TORQUE) {
        sensorOffset += 6 * m_sensors.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
    }

    if (type > iDynTree::ACCELEROMETER) {
        sensorOffset += 3 * m_sensors.getNrOfSensors(iDynTree::ACCELEROMETER);
    }

    if (type > iDynTree::GYROSCOPE) {
        sensorOffset += 3 * m_sensors.getNrOfSensors(iDynTree::GYROSCOPE);
    }

    if (type > iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER) {
        sensorOffset += 3 * m_sensors.getNrOfSensors(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER);
    }

    if (type > iDynTree::THREE_AXIS_FORCE_TORQUE_CONTACT) {
        sensorOffset += 3 * m_sensors.getNrOfSensors(iDynTree::THREE_AXIS_FORCE_TORQUE_CONTACT);
    }

    sensorOffset += sensorIdx * getSensorTypeSize(type);

    iDynTree::IndexRange ret;
    ret.offset = sensorOffset;
    ret.size = getSensorTypeSize(type);

    return ret;
}

iDynTree::IndexRange BerdyHelper::getRangeDOFSensorVariable(const BerdySensorTypes sensorType,
                                                            const iDynTree::DOFIndex idx) const
{
    iDynTree::IndexRange ret = iDynTree::IndexRange::InvalidRange();
    ret.size = 1;

    if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        // For ORIGINAL_BERDY_FIXED_BASE we know  that iDynTree::DOFIndex is always equalt to
        // JointIndex
        iDynTree::TraversalIndex trvIdx = getTraversalIndexFromJointIndex(
            m_model, m_dynamicsTraversal, (iDynTree::JointIndex) idx);

        if (sensorType == DOF_ACCELERATION_SENSOR) {
            ret.offset = berdySensorTypeOffsets.dofAccelerationOffset + (trvIdx - 1);
        }

        if (sensorType == DOF_TORQUE_SENSOR) {
            ret.offset = berdySensorTypeOffsets.dofTorquesOffset + (trvIdx - 1);
        }
    }
    else {
        assert(m_options.berdyVariant == BERDY_FLOATING_BASE);

        if (sensorType == DOF_ACCELERATION_SENSOR) {
            ret.offset = berdySensorTypeOffsets.dofAccelerationOffset + idx;
        }

        if (sensorType == DOF_TORQUE_SENSOR) {
            ret.offset = berdySensorTypeOffsets.dofTorquesOffset + idx;
        }
    }

    return ret;
}

iDynTree::IndexRange BerdyHelper::getRangeJointSensorVariable(const BerdySensorTypes sensorType,
                                                              const iDynTree::JointIndex idx) const
{
    iDynTree::IndexRange ret = iDynTree::IndexRange::InvalidRange();
    ret.size = 6;

    if (sensorType == JOINT_WRENCH_SENSOR) {
        ret.offset = berdySensorsInfo.jntIdxToOffset[idx];
    }

    return ret;
}

iDynTree::IndexRange BerdyHelper::getRangeLinkSensorVariable(const BerdySensorTypes sensorType,
                                                             const iDynTree::LinkIndex idx) const
{
    iDynTree::IndexRange ret = iDynTree::IndexRange::InvalidRange();
    ret.size = 6;

    if (sensorType == NET_EXT_WRENCH_SENSOR) {
        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
            iDynTree::TraversalIndex trvIdx =
                m_dynamicsTraversal.getTraversalIndexFromLinkIndex(idx);

            if (m_options.includeFixedBaseExternalWrench) {
                ret.offset = berdySensorTypeOffsets.netExtWrenchOffset + 6 * trvIdx;
            }
            else {
                ret.offset = berdySensorTypeOffsets.netExtWrenchOffset + 6 * (trvIdx - 1);
            }
        }
        else {
            assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
            ret.offset = berdySensorTypeOffsets.netExtWrenchOffset + 6 * idx;
        }
    }

    assert(ret.isValid());
    return ret;
}

iDynTree::IndexRange
BerdyHelper::getRangeLinkProperAccDynEqFixedBase(const iDynTree::LinkIndex idx) const
{
    iDynTree::IndexRange ret;

    if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        iDynTree::TraversalIndex trvIdx = m_dynamicsTraversal.getTraversalIndexFromLinkIndex(idx);
        ret.offset = 19 * (trvIdx - 1) + 0;
        ret.size = 6;
        return ret;
    }

    return iDynTree::IndexRange::InvalidRange();
}

iDynTree::IndexRange
BerdyHelper::getRangeLinkNetTotalWrenchDynEqFixedBase(const iDynTree::LinkIndex idx) const
{
    iDynTree::IndexRange ret;

    if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        iDynTree::TraversalIndex trvIdx = m_dynamicsTraversal.getTraversalIndexFromLinkIndex(idx);
        ret.offset = 19 * (trvIdx - 1) + 6;
        ret.size = 6;
        return ret;
    }

    return iDynTree::IndexRange::InvalidRange();
}

iDynTree::IndexRange
BerdyHelper::getRangeJointWrenchDynEqFixedBase(const iDynTree::JointIndex idx) const
{
    iDynTree::IndexRange ret;

    if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        iDynTree::TraversalIndex trvIdx =
            getTraversalIndexFromJointIndex(m_model, m_dynamicsTraversal, idx);
        ret.offset = 19 * (trvIdx - 1) + 12;
        ret.size = 6;
        return ret;
    }

    return iDynTree::IndexRange::InvalidRange();
}

iDynTree::IndexRange
BerdyHelper::getRangeDOFTorqueDynEqFixedBase(const iDynTree::DOFIndex idx) const
{
    iDynTree::IndexRange ret;

    if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        iDynTree::TraversalIndex trvIdx =
            getTraversalIndexFromJointIndex(m_model, m_dynamicsTraversal, idx);
        ret.offset = 19 * (trvIdx - 1) + 18;
        ret.size = 1;
        return ret;
    }

    return iDynTree::IndexRange::InvalidRange();
}

iDynTree::IndexRange
BerdyHelper::getRangeAccelerationPropagationFloatingBase(const iDynTree::JointIndex idx) const
{
    assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
    iDynTree::IndexRange ret;
    ret.offset = 6 * m_model.getNrOfLinks() + 6 * idx;
    ret.size = 6;
    return ret;
}

iDynTree::IndexRange
BerdyHelper::getRangeNewtonEulerEquationsFloatingBase(const iDynTree::LinkIndex idx) const
{
    assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
    iDynTree::IndexRange ret;
    ret.offset = 6 * idx;
    ret.size = 6;
    return ret;
}

bool BerdyHelper::computeBerdyDynamicsMatricesFixedBase(
    iDynTree::SparseMatrix<iDynTree::ColumnMajor>& D,
    iDynTree::VectorDynSize& bD)
{
    D.resize(m_nrOfDynamicEquations, m_nrOfDynamicalVariables);
    bD.resize(m_nrOfDynamicEquations);
    // TODO: \todo check if this is a bottleneck
    matrixDElements.clear();
    bD.zero();

    // We follow the traversal skipping the base because all the equations of the RNEA are not
    // directly affecting the base
    for (iDynTree::TraversalIndex traversalEl = 1;
         traversalEl
         < static_cast<iDynTree::TraversalIndex>(m_dynamicsTraversal.getNrOfVisitedLinks());
         traversalEl++) {
        iDynTree::LinkConstPtr visitedLink = m_dynamicsTraversal.getLink(traversalEl);
        iDynTree::LinkConstPtr parentLink = m_dynamicsTraversal.getParentLink(traversalEl);
        iDynTree::IJointConstPtr toParentJoint = m_dynamicsTraversal.getParentJoint(traversalEl);
        iDynTree::LinkIndex visitedLinkIdx = visitedLink->getIndex();
        iDynTree::LinkIndex parentLinkIdx = parentLink->getIndex();

        ///////////////////////////////////////////////////////////
        // Acceleration forward kinematics propagation (D part)
        ///////////////////////////////////////////////////////////
        const iDynTree::Transform& visited_X_parent =
            toParentJoint->getTransform(m_jointPos, visitedLinkIdx, parentLinkIdx);

        // Proper acc propagation equations
        matrixDElements.addDiagonalMatrix(
            getRangeLinkProperAccDynEqFixedBase(visitedLinkIdx),
            getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION, visitedLinkIdx),
            -1);

        // This should not be added if the variant is ORIGINAL_BERDY_FIXED_BASE and the parentLink
        // is the base because in that case the acceleration of the base is not a dynamic variable
        if (!(m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE
              && parentLinkIdx == m_dynamicsTraversal.getBaseLink()->getIndex())) {
            matrixDElements.addSubMatrix(
                getRangeLinkProperAccDynEqFixedBase(visitedLinkIdx).offset,
                getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION, parentLinkIdx).offset,
                visited_X_parent.asAdjointTransform());
        }

        size_t jointDOFs = toParentJoint->getNrOfDOFs();
        size_t dofOffset = toParentJoint->getDOFsOffset();

        for (size_t localDof = 0; localDof < jointDOFs; localDof++) {
            iDynTree::SpatialMotionVector S =
                toParentJoint->getMotionSubspaceVector(localDof, visitedLinkIdx, parentLinkIdx);
            iDynTree::Matrix6x1 SdynTree;
            toEigen(SdynTree) = toEigen(S);

            // From S to iDynTreeMatrix
            matrixDElements.addSubMatrix(
                getRangeLinkProperAccDynEqFixedBase(visitedLinkIdx).offset,
                getRangeDOFVariable(DOF_ACCELERATION, dofOffset + localDof).offset,
                SdynTree);
        }

        ///////////////////////////////////////////////////////////
        // Acceleration forward kinematics propagation (bD part)
        ///////////////////////////////////////////////////////////
        iDynTree::SpatialMotionVector biasAcc;
        biasAcc.zero();

        for (size_t localDof = 0; localDof < jointDOFs; localDof++) {
            iDynTree::SpatialMotionVector S =
                toParentJoint->getMotionSubspaceVector(localDof, visitedLinkIdx, parentLinkIdx);
            biasAcc =
                biasAcc + m_linkVels(visitedLinkIdx).cross(S * m_jointVel(dofOffset + localDof));
        }

        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE
            && parentLinkIdx == m_dynamicsTraversal.getBaseLink()->getIndex()) {
            // If the variant is ORIGINAL_BERDY_FIXED_BASE variant, we need to account explicitly
            // for the gravity for links whose parent is the base
            biasAcc = biasAcc - visited_X_parent * m_gravity6D;
        }

        setSubVector(bD, getRangeLinkProperAccDynEqFixedBase(visitedLinkIdx), toEigen(biasAcc));

        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
            ///////////////////////////////////////////////////////////
            // Total net wrench without gravity (D part, only for ORIGINAL_BERDY_FIXED_BASE
            ///////////////////////////////////////////////////////////
            matrixDElements.addDiagonalMatrix(
                getRangeLinkNetTotalWrenchDynEqFixedBase(visitedLinkIdx),
                getRangeLinkVariable(NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV, visitedLinkIdx),
                -1);

            matrixDElements.addSubMatrix(
                getRangeLinkNetTotalWrenchDynEqFixedBase(visitedLinkIdx).offset,
                getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION, visitedLinkIdx).offset,
                visitedLink->getInertia().asMatrix());

            ///////////////////////////////////////////////////////////
            // Total net wrench without gravity (bD part, only for ORIGINAL_BERDY_FIXED_BASE
            ///////////////////////////////////////////////////////////
            setSubVector(bD,
                         getRangeLinkNetTotalWrenchDynEqFixedBase(visitedLinkIdx),
                         toEigen(m_linkVels(visitedLinkIdx)
                                 * (visitedLink->getInertia() * m_linkVels(visitedLinkIdx))));
        }

        ///////////////////////////////////////////////////////////
        // Joint wrench (D part)
        ///////////////////////////////////////////////////////////

        // Joint wrench itself
        matrixDElements.addDiagonalMatrix(
            getRangeJointWrenchDynEqFixedBase(toParentJoint->getIndex()),
            getRangeJointVariable(JOINT_WRENCH, toParentJoint->getIndex()),
            -1);

        // Wrench of child links
        // Iterate on childs of visitedLink
        // We obtain all the children as all the neighbors of the link, except
        // for its parent
        // \todo TODO this point is definitly Tree-specific
        // \todo TODO this "get child" for is duplicated in the code, we
        //            should try to consolidate it
        for (unsigned int neigh_i = 0; neigh_i < m_model.getNrOfNeighbors(visitedLinkIdx);
             neigh_i++) {
            iDynTree::LinkIndex neighborIndex =
                m_model.getNeighbor(visitedLinkIdx, neigh_i).neighborLink;
            if (!parentLink || neighborIndex != parentLink->getIndex()) {
                iDynTree::LinkIndex childIndex = neighborIndex;
                iDynTree::IJointConstPtr neighborJoint =
                    m_model.getJoint(m_model.getNeighbor(visitedLinkIdx, neigh_i).neighborJoint);
                const iDynTree::Transform& visitedLink_X_child =
                    neighborJoint->getTransform(m_jointPos, visitedLinkIdx, childIndex);

                matrixDElements.addSubMatrix(
                    getRangeJointWrenchDynEqFixedBase(toParentJoint->getIndex()).offset,
                    getRangeJointVariable(JOINT_WRENCH, neighborJoint->getIndex()).offset,
                    visitedLink_X_child.asAdjointTransformWrench());
            }
        }

        // Net external wrench
        if (m_options.includeAllNetExternalWrenchesAsDynamicVariables) {
            matrixDElements.addDiagonalMatrix(
                getRangeJointWrenchDynEqFixedBase(toParentJoint->getIndex()),
                getRangeLinkVariable(NET_EXT_WRENCH, visitedLinkIdx),
                -1);
        }

        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
            // In the ORIGINAL_BERDY_FIXED_BASE, we also add to the rows of the joint wrenches
            // the depend on the total wrenches
            matrixDElements.addDiagonalMatrix(
                getRangeJointWrenchDynEqFixedBase(toParentJoint->getIndex()),
                getRangeLinkVariable(NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV, visitedLinkIdx),
                1);
        }

        if (m_options.berdyVariant == BERDY_FLOATING_BASE) {
            // In the floating base variant, we embed all the inertia related terms directly in the
            // floating base
            matrixDElements.addSubMatrix(
                getRangeJointWrenchDynEqFixedBase(toParentJoint->getIndex()).offset,
                getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION, visitedLinkIdx).offset,
                visitedLink->getInertia().asMatrix());
        }

        ///////////////////////////////////////////////////////////
        // Joint wrench (bD part, only for BERDY_FLOATING_BASE)
        ///////////////////////////////////////////////////////////
        if (m_options.berdyVariant == BERDY_FLOATING_BASE) {
            setSubVector(
                bD,
                getRangeJointWrenchDynEqFixedBase(toParentJoint->getIndex()),
                toEigen(m_linkVels(visitedLinkIdx)
                            .cross(visitedLink->getInertia() * m_linkVels(visitedLinkIdx))));
        }

        ////////////////////////////////////////////////////////////
        /// DOF torques equations (D part, bD is always zero).
        /// Only for ORIGINAL_BERDY_FIXED_BASE, BERDY_FLOATING_BASE does not
        // include the torques as dynamic variables
        ////////////////////////////////////////////////////////////
        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
            for (size_t localDof = 0; localDof < jointDOFs; localDof++) {
                iDynTree::SpatialMotionVector S =
                    toParentJoint->getMotionSubspaceVector(localDof, visitedLinkIdx, parentLinkIdx);
                iDynTree::Matrix1x6 SdynTree;
                iDynTree::toEigen(SdynTree) = toEigen(S).transpose();

                matrixDElements.addSubMatrix(
                    getRangeDOFTorqueDynEqFixedBase(dofOffset + localDof).offset,
                    getRangeJointVariable(JOINT_WRENCH, toParentJoint->getIndex()).offset,
                    SdynTree);

                matrixDElements.pushTriplet(
                    iDynTree::Triplet(getRangeDOFTorqueDynEqFixedBase(dofOffset + localDof).offset,
                                      getRangeDOFVariable(DOF_TORQUE, dofOffset + localDof).offset,
                                      -1));
            }
        }
    }

    D.setFromTriplets(matrixDElements);
    return true;
}

iDynTree::Matrix6x1
BerdyHelper::getBiasTermJointAccelerationPropagation(iDynTree::IJointConstPtr joint,
                                                     const iDynTree::LinkIndex parentLinkIdx,
                                                     const iDynTree::LinkIndex childLinkIdx,
                                                     const iDynTree::Transform& child_X_parent)
{
    iDynTree::Matrix6x1 biasTerm;
    biasTerm.zero();

    auto biasTermEig = toEigen(biasTerm);

    // Implementing equation 36 in https://www.sharelatex.com/project/59439ec80788901873a803fb
    iDynTree::Vector3 parentBodyAngularVel = m_linkVels(parentLinkIdx).getAngularVec3();

    iDynTree::Rotation child_R_parent = child_X_parent.getRotation();
    iDynTree::Transform parent_X_child = child_X_parent.inverse();
    iDynTree::Position parent_o_child = parent_X_child.getPosition();
    iDynTree::Rotation parent_R_child = parent_X_child.getRotation();
    iDynTree::Transform mixed_X_leftTrivialized =
        iDynTree::Transform(parent_R_child, iDynTree::Position::Zero());

    // Compute the mixed relative velocity of the child w.r.t to the parent
    iDynTree::Twist relJointVelInMixed = iDynTree::Twist::Zero();
    for (size_t localDof = 0; localDof < joint->getNrOfDOFs(); localDof++) {
        iDynTree::SpatialMotionVector SleftTrivialzed =
            joint->getMotionSubspaceVector(localDof, childLinkIdx, parentLinkIdx);
        iDynTree::SpatialMotionVector Smixed = mixed_X_leftTrivialized * SleftTrivialzed;

        relJointVelInMixed =
            relJointVelInMixed + (Smixed * m_jointVel(joint->getDOFsOffset() + localDof));
    }

    // Compute the first term
    biasTermEig.segment<3>(0) =
        2 * toEigen(child_R_parent)
        * (toEigen(parentBodyAngularVel).cross(toEigen(relJointVelInMixed.getLinearVec3())));
    biasTermEig.segment<3>(3) =
        toEigen(child_R_parent)
        * (toEigen(parentBodyAngularVel).cross(toEigen(relJointVelInMixed.getAngularVec3())));

    // Compute the second term
    biasTermEig.segment<3>(0) +=
        toEigen(child_R_parent)
        * (toEigen(parentBodyAngularVel)
               .cross(toEigen(parentBodyAngularVel).cross(toEigen(parent_o_child))));

    return biasTerm;
}

bool BerdyHelper::computeBerdyDynamicsMatricesFloatingBase(
    iDynTree::SparseMatrix<iDynTree::ColumnMajor>& D,
    iDynTree::VectorDynSize& bD)
{
    D.resize(m_nrOfDynamicEquations, m_nrOfDynamicalVariables);
    bD.resize(m_nrOfDynamicEquations);
    // TODO: \todo check if this is a bottleneck
    matrixDElements.clear();
    bD.zero();

    // Add the equation of the Newton-Euler for a link
    for (iDynTree::LinkIndex lnkIdx = 0;
         lnkIdx < static_cast<iDynTree::LinkIndex>(m_model.getNrOfLinks());
         lnkIdx++) {
        iDynTree::LinkConstPtr link = m_model.getLink(lnkIdx);

        // Term depending on the sensor acceleration
        matrixDElements.addSubMatrix(
            getRangeNewtonEulerEquationsFloatingBase(lnkIdx).offset,
            getRangeLinkVariable(LINK_BODY_PROPER_CLASSICAL_ACCELERATION, lnkIdx).offset,
            link->getInertia().asMatrix());

        // Term depending on external force-torque
        matrixDElements.addDiagonalMatrix(getRangeNewtonEulerEquationsFloatingBase(lnkIdx),
                                          getRangeLinkVariable(NET_EXT_WRENCH, lnkIdx),
                                          -1);

        // Term depending on the force exchanged with the parent link, if this link is not the base
        // of the traversal
        iDynTree::LinkIndex parentLinkIdx = iDynTree::LINK_INVALID_INDEX;
        if (lnkIdx != m_dynamicsTraversal.getBaseLink()->getIndex()) {
            iDynTree::JointIndex parentJointIdx =
                m_dynamicsTraversal.getParentJointFromLinkIndex(lnkIdx)->getIndex();
            parentLinkIdx = m_dynamicsTraversal.getParentLinkFromLinkIndex(lnkIdx)->getIndex();
            matrixDElements.addDiagonalMatrix(getRangeNewtonEulerEquationsFloatingBase(lnkIdx),
                                              getRangeJointVariable(JOINT_WRENCH, parentJointIdx),
                                              -1);
        }

        // Term depending on the force exchanged with the children links
        for (unsigned int neigh_i = 0; neigh_i < m_model.getNrOfNeighbors(lnkIdx); neigh_i++) {
            iDynTree::LinkIndex neighborIndex = m_model.getNeighbor(lnkIdx, neigh_i).neighborLink;
            if (neighborIndex != parentLinkIdx) {
                iDynTree::LinkIndex childIndex = neighborIndex;
                iDynTree::IJointConstPtr neighborJoint =
                    m_model.getJoint(m_model.getNeighbor(lnkIdx, neigh_i).neighborJoint);
                const iDynTree::Transform& visitedLink_X_child =
                    neighborJoint->getTransform(m_jointPos, lnkIdx, childIndex);

                matrixDElements.addSubMatrix(
                    getRangeNewtonEulerEquationsFloatingBase(lnkIdx).offset,
                    getRangeJointVariable(JOINT_WRENCH, neighborJoint->getIndex()).offset,
                    visitedLink_X_child.asAdjointTransformWrench());
            }
        }

        // bias Term
        iDynTree::Twist angularPartOfLeftTrivializedVel = iDynTree::Twist(
            iDynTree::LinearMotionVector3(0.0, 0.0, 0.0), m_linkVels(lnkIdx).getAngularVec3());
        setSubVector(bD,
                     getRangeNewtonEulerEquationsFloatingBase(lnkIdx),
                     iDynTree::toEigen(angularPartOfLeftTrivializedVel.cross(
                         link->getInertia() * angularPartOfLeftTrivializedVel)));
    }

    // Add the equation of the kinematic propagation of sensor acceleration
    for (iDynTree::JointIndex jntIdx = 0;
         jntIdx < static_cast<iDynTree::JointIndex>(m_model.getNrOfJoints());
         jntIdx++) {
        // This equation is one of the few place in which we use the parent-child relations
        iDynTree::LinkIndex childLinkIdx =
            m_dynamicsTraversal.getChildLinkIndexFromJointIndex(m_model, jntIdx);
        iDynTree::LinkIndex parentLinkIdx =
            m_dynamicsTraversal.getParentLinkIndexFromJointIndex(m_model, jntIdx);

        matrixDElements.addDiagonalMatrix(
            getRangeAccelerationPropagationFloatingBase(jntIdx),
            getRangeLinkVariable(LINK_BODY_PROPER_CLASSICAL_ACCELERATION, childLinkIdx),
            -1);

        iDynTree::IJointConstPtr joint = m_model.getJoint(jntIdx);
        const iDynTree::Transform& child_X_parent =
            joint->getTransform(m_jointPos, childLinkIdx, parentLinkIdx);

        matrixDElements.addSubMatrix(
            getRangeAccelerationPropagationFloatingBase(jntIdx).offset,
            getRangeLinkVariable(LINK_BODY_PROPER_CLASSICAL_ACCELERATION, parentLinkIdx).offset,
            child_X_parent.asAdjointTransform());

        // This for automatically handles joint with any number of DOFs
        for (size_t localDof = 0; localDof < joint->getNrOfDOFs(); localDof++) {
            iDynTree::SpatialMotionVector S =
                joint->getMotionSubspaceVector(localDof, childLinkIdx, parentLinkIdx);
            iDynTree::Matrix6x1 SdynTree;
            toEigen(SdynTree) = toEigen(S);

            matrixDElements.addSubMatrix(
                getRangeAccelerationPropagationFloatingBase(jntIdx).offset,
                getRangeDOFVariable(DOF_ACCELERATION, joint->getDOFsOffset() + localDof).offset,
                SdynTree);
        }

        // The known term for kinematic calibration is quite complicate due to the use of sensor
        // proper acceleration, for this reason it is implemented in a separate function
        iDynTree::Matrix6x1 biasTerm = getBiasTermJointAccelerationPropagation(
            joint, parentLinkIdx, childLinkIdx, child_X_parent);
        setSubVector(bD, getRangeAccelerationPropagationFloatingBase(jntIdx), toEigen(biasTerm));
    }

    D.setFromTriplets(matrixDElements);
    return true;
}

bool BerdyHelper::computeBerdySensorMatrices(iDynTree::SparseMatrix<iDynTree::ColumnMajor>& Y,
                                             iDynTree::VectorDynSize& bY)
{
    Y.resize(m_nrOfSensorsMeasurements, m_nrOfDynamicalVariables);
    bY.resize(m_nrOfSensorsMeasurements);
    // \todo TODO check if this is a bottleneck
    //    Y.zero();
    matrixYElements.clear();
    bY.zero();

    // For now handle all sensor types explicitly TODO clean this up

    ////////////////////////////////////////////////////////////////////////
    ///// SIX AXIS F/T SENSORS
    ////////////////////////////////////////////////////////////////////////
    size_t numOfFTs = m_sensors.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
    for (size_t idx = 0; idx < numOfFTs; idx++) {
        iDynTree::SixAxisForceTorqueSensor* ftSens =
            (iDynTree::SixAxisForceTorqueSensor*) m_sensors.getSensor(
                iDynTree::SIX_AXIS_FORCE_TORQUE, idx);
        iDynTree::LinkIndex childLink;
        childLink = m_dynamicsTraversal.getChildLinkIndexFromJointIndex(
            m_model, ftSens->getParentJointIndex());
        iDynTree::Matrix6x6 sensor_M_link;
        ftSens->getWrenchAppliedOnLinkInverseMatrix(childLink, sensor_M_link);
        iDynTree::IndexRange sensorRange =
            this->getRangeSensorVariable(iDynTree::SIX_AXIS_FORCE_TORQUE, idx);
        iDynTree::IndexRange jointWrenchRange =
            this->getRangeJointVariable(JOINT_WRENCH, ftSens->getParentJointIndex());

        matrixYElements.addSubMatrix(sensorRange.offset, jointWrenchRange.offset, sensor_M_link);
        // bY for the F/T sensor is equal to zero
    }

    ////////////////////////////////////////////////////////////////////////
    ///// ACCELEROMETERS
    ////////////////////////////////////////////////////////////////////////
    unsigned int numAccl = m_sensors.getNrOfSensors(iDynTree::ACCELEROMETER);
    for (size_t idx = 0; idx < numAccl; idx++) {
        iDynTree::AccelerometerSensor* accelerometer =
            (iDynTree::AccelerometerSensor*) m_sensors.getSensor(iDynTree::ACCELEROMETER, idx);
        iDynTree::LinkIndex parentLinkId = accelerometer->getParentLinkIndex();
        iDynTree::Transform sensor_X_link = accelerometer->getLinkSensorTransform().inverse();
        iDynTree::IndexRange sensorRange =
            this->getRangeSensorVariable(iDynTree::ACCELEROMETER, idx);

        // The sensor equation are different between ORIGINAL_BERDY_FIXED_BASE and
        // BERDY_FLOATING_BASE, due to the difference in the convention used for expressing the body
        // acceleration
        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
            // Y(sensorRange,linkBodyProperAcRange) for the accelerometer is the first three rows of
            // the sensor_X_link adjoint matrix
            iDynTree::MatrixFixSize<3, 6> xLinkLinear;
            iDynTree::toEigen(xLinkLinear) =
                toEigen(sensor_X_link.asAdjointTransform()).block<3, 6>(0, 0);

            iDynTree::IndexRange linkBodyProperAcRange =
                this->getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION, parentLinkId);
            matrixYElements.addSubMatrix(
                sensorRange.offset, linkBodyProperAcRange.offset, xLinkLinear);

            iDynTree::Twist vSensor = sensor_X_link * m_linkVels(parentLinkId);

            // bY for the accelerometer is the cross product of the angular part and the linear part
            // of the body 6D velocity expressed in the sensor orientation and wrt to the
            setSubVector(
                bY,
                sensorRange,
                iDynTree::toEigen(vSensor.getAngularVec3().cross(vSensor.getLinearVec3())));
        }
        else {
            assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
            // Y(sensorRange,linkBodyProperAcRange) for the accelerometer is the first three rows of
            // the sensor_X_link adjoint matrix
            iDynTree::MatrixFixSize<3, 6> xLinkLinear;
            iDynTree::toEigen(xLinkLinear) =
                toEigen(sensor_X_link.asAdjointTransform()).block<3, 6>(0, 0);

            iDynTree::IndexRange linkBodyProperAcRange =
                this->getRangeLinkVariable(LINK_BODY_PROPER_CLASSICAL_ACCELERATION, parentLinkId);
            matrixYElements.addSubMatrix(
                sensorRange.offset, linkBodyProperAcRange.offset, xLinkLinear);

            iDynTree::Position link_o_sensor = sensor_X_link.inverse().getPosition();
            iDynTree::Rotation sensor_R_link = sensor_X_link.getRotation();

            // bY for the accelerometer
            Eigen::Vector3d linkAngVel =
                iDynTree::toEigen(m_linkVels(parentLinkId).getAngularVec3());
            setSubVector(bY,
                         sensorRange,
                         iDynTree::toEigen(sensor_R_link)
                             * (linkAngVel.cross(linkAngVel.cross(toEigen(link_o_sensor)))));
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// GYROSCOPES
    ////////////////////////////////////////////////////////////////////////
    unsigned int numGyro = m_sensors.getNrOfSensors(iDynTree::GYROSCOPE);
    for (size_t idx = 0; idx < numGyro; idx++) {
        iDynTree::GyroscopeSensor* gyroscope =
            (iDynTree::GyroscopeSensor*) m_sensors.getSensor(iDynTree::GYROSCOPE, idx);
        iDynTree::LinkIndex parentLinkId = gyroscope->getParentLinkIndex();
        iDynTree::Transform sensor_X_link = gyroscope->getLinkSensorTransform().inverse();
        iDynTree::IndexRange sensorRange = this->getRangeSensorVariable(iDynTree::GYROSCOPE, idx);

        // Y for the gyroscope is already zero

        // bY for the gyroscope is just the angular velocity of the link rotated in the sensor frame
        setSubVector(
            bY, sensorRange, toEigen((sensor_X_link * m_linkVels(parentLinkId)).getAngularVec3()));
    }

    ////////////////////////////////////////////////////////////////////////
    ///// THREE AXIS ANGULAR ACCELEROMETERS
    ////////////////////////////////////////////////////////////////////////
    unsigned int numThreeAxisAngularAccelerometer =
        m_sensors.getNrOfSensors(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER);
    for (size_t idx = 0; idx < numThreeAxisAngularAccelerometer; idx++) {
        iDynTree::ThreeAxisAngularAccelerometerSensor* angAccelerometer =
            (iDynTree::ThreeAxisAngularAccelerometerSensor*) m_sensors.getSensor(
                iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER, idx);
        iDynTree::LinkIndex parentLinkId = angAccelerometer->getParentLinkIndex();
        iDynTree::Transform sensor_X_link = angAccelerometer->getLinkSensorTransform().inverse();
        iDynTree::IndexRange sensorRange =
            this->getRangeSensorVariable(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER, idx);

        // Y for the gyroscope is just a rotation between the link frame and the sensor frame
        // for all the rappresentations of acceleration
        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
            iDynTree::IndexRange linkBodyProperAcRange =
                this->getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION, parentLinkId);
            matrixYElements.addSubMatrix(
                sensorRange.offset, linkBodyProperAcRange.offset + 3, sensor_X_link.getRotation());
        }
        else {
            assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
            iDynTree::IndexRange linkBodyProperAcRange =
                this->getRangeLinkVariable(LINK_BODY_PROPER_CLASSICAL_ACCELERATION, parentLinkId);
            matrixYElements.addSubMatrix(
                sensorRange.offset, linkBodyProperAcRange.offset + 3, sensor_X_link.getRotation());
        }

        // bY for the angular accelerometer is zero
    }

    ////////////////////////////////////////////////////////////////////////
    ///// THREE AXIS FORCE TORQUE CONTACT
    ////////////////////////////////////////////////////////////////////////
    unsigned int numThreeAxisForceTorqueContact =
        m_sensors.getNrOfSensors(iDynTree::THREE_AXIS_FORCE_TORQUE_CONTACT);
    for (size_t idx = 0; idx < numThreeAxisForceTorqueContact; idx++) {
        iDynTree::ThreeAxisForceTorqueContactSensor* threeAxisFTContactSensor =
            (iDynTree::ThreeAxisForceTorqueContactSensor*) m_sensors.getSensor(
                iDynTree::THREE_AXIS_FORCE_TORQUE_CONTACT, idx);
        iDynTree::LinkIndex parentLinkId = threeAxisFTContactSensor->getParentLinkIndex();
        iDynTree::Transform sensor_X_link =
            threeAxisFTContactSensor->getLinkSensorTransform().inverse();
        iDynTree::IndexRange sensorRange =
            this->getRangeSensorVariable(iDynTree::THREE_AXIS_FORCE_TORQUE_CONTACT, idx);
        iDynTree::IndexRange linkNetExtForceTorque =
            this->getRangeLinkVariable(NET_EXT_WRENCH, parentLinkId);

        // Y is just the third (force on z) fourth and fifth (torque on x, y) component of the
        // Transform
        iDynTree::MatrixFixSize<3, 6> threeAxisFTtransform;
        iDynTree::toEigen(threeAxisFTtransform) =
            toEigen(sensor_X_link.asAdjointTransform()).block<3, 6>(2, 0);

        matrixYElements.addSubMatrix(
            sensorRange.offset, linkNetExtForceTorque.offset, threeAxisFTtransform);

        // bY is zero
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT ACCELERATIONS
    ////////////////////////////////////////////////////////////////////////
    if (m_options.includeAllJointAccelerationsAsSensors) {
        for (iDynTree::DOFIndex idx = 0;
             idx < static_cast<iDynTree::DOFIndex>(m_model.getNrOfDOFs());
             idx++) {
            // Y for the joint accelerations is just a rows of 0s and one 1  corresponding to the
            // location of the relative joint acceleration in the dynamic variables vector
            iDynTree::IndexRange sensorRange =
                this->getRangeDOFSensorVariable(DOF_ACCELERATION_SENSOR, idx);
            iDynTree::IndexRange jointAccRange = this->getRangeDOFVariable(DOF_ACCELERATION, idx);

            matrixYElements.pushTriplet(
                iDynTree::Triplet(sensorRange.offset, jointAccRange.offset, 1));

            // bY for the joint acceleration is zero
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT TORQUES
    ////////////////////////////////////////////////////////////////////////
    if (m_options.includeAllJointTorquesAsSensors) {
        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
            for (iDynTree::DOFIndex idx = 0;
                 idx < static_cast<iDynTree::DOFIndex>(m_model.getNrOfDOFs());
                 idx++) {
                // Y for the joint torques is just a rows of 0s and one 1  corresponding to the
                // location
                // of the relative joint torques in the dynamic variables vector
                iDynTree::IndexRange sensorRange =
                    this->getRangeDOFSensorVariable(DOF_TORQUE_SENSOR, idx);
                iDynTree::IndexRange jointTrqRange = this->getRangeDOFVariable(DOF_TORQUE, idx);

                matrixYElements.pushTriplet(
                    iDynTree::Triplet(sensorRange.offset, jointTrqRange.offset, 1));

                // bY for the joint torques is zero
            }
        }
        else {
            assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
            // We have to map the joint wrench to the joint torques, since joint wrench is
            // considered as dynamic variable
            for (iDynTree::TraversalIndex traversalEl = 1;
                 traversalEl
                 < static_cast<iDynTree::TraversalIndex>(m_dynamicsTraversal.getNrOfVisitedLinks());
                 traversalEl++) {
                iDynTree::LinkConstPtr visitedLink = m_dynamicsTraversal.getLink(traversalEl);
                iDynTree::LinkConstPtr parentLink = m_dynamicsTraversal.getParentLink(traversalEl);
                iDynTree::IJointConstPtr toParentJoint =
                    m_dynamicsTraversal.getParentJoint(traversalEl);
                iDynTree::LinkIndex visitedLinkIdx = visitedLink->getIndex();
                iDynTree::LinkIndex parentLinkIdx = parentLink->getIndex();

                size_t jointDOFs = toParentJoint->getNrOfDOFs();
                size_t dofOffset = toParentJoint->getDOFsOffset();

                for (size_t localDof = 0; localDof < jointDOFs; localDof++) {
                    iDynTree::SpatialMotionVector S = toParentJoint->getMotionSubspaceVector(
                        localDof, visitedLinkIdx, parentLinkIdx);
                    iDynTree::Matrix1x6 SdynTree;
                    iDynTree::toEigen(SdynTree) = toEigen(S).transpose();

                    matrixYElements.addSubMatrix(
                        getRangeDOFSensorVariable(DOF_TORQUE_SENSOR, dofOffset + localDof).offset,
                        getRangeJointVariable(JOINT_WRENCH, toParentJoint->getIndex()).offset,
                        SdynTree);
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// NET EXTERNAL WRENCHES ACTING ON LINKS
    ////////////////////////////////////////////////////////////////////////
    if (m_options.includeAllNetExternalWrenchesAsSensors) {
        for (iDynTree::LinkIndex idx = 0;
             idx < static_cast<iDynTree::LinkIndex>(m_model.getNrOfLinks());
             idx++) {
            // If this link is the (fixed) base link and the
            // berdy variant is ORIGINAL_BERDY_FIXED_BASE , then
            // the net wrench applied on the base is not part of the dynamical
            // system. Anyhow, we can still write the base wrench as a function
            // of sum of the joint wrenches of all the joints attached to the base (tipically just
            // one)
            if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE
                && m_dynamicsTraversal.getBaseLink()->getIndex() == idx) {
                if (m_options.includeFixedBaseExternalWrench) {
                    // Y encodes the (time varyng) relation between the wrenches transmitted by the
                    // joint attached to the base and net external wrench applied on the robot \todo
                    // TODO this "get child" for is duplicated in the code, we should try to
                    // consolidate it
                    for (unsigned int neigh_i = 0; neigh_i < m_model.getNrOfNeighbors(idx);
                         neigh_i++) {
                        iDynTree::LinkIndex neighborIndex =
                            m_model.getNeighbor(idx, neigh_i).neighborLink;
                        iDynTree::LinkIndex childIndex = neighborIndex;
                        iDynTree::IJointConstPtr neighborJoint =
                            m_model.getJoint(m_model.getNeighbor(idx, neigh_i).neighborJoint);
                        const iDynTree::Transform& base_X_child =
                            neighborJoint->getTransform(m_jointPos, idx, childIndex);
                        iDynTree::Transform measurementFrame_X_child =
                            m_link_H_externalWrenchMeasurementFrame[idx].inverse() * base_X_child;

                        matrixYElements.addSubMatrix(
                            getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR, idx).offset,
                            getRangeJointVariable(JOINT_WRENCH, neighborJoint->getIndex()).offset,
                            measurementFrame_X_child.asAdjointTransformWrench());
                    }

                    // bY encodes the weight of the base link due to gravity (we omit the v*I*v as
                    // it is always zero)
                    iDynTree::Wrench baseLinkNetTotalWrenchesWithoutGrav =
                        -(m_model.getLink(idx)->getInertia() * m_gravity6D);
                    setSubVector(bY,
                                 getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR, idx),
                                 iDynTree::toEigen(baseLinkNetTotalWrenchesWithoutGrav));
                }
            }
            else {
                // Y for the net external wrenches is just
                // six rows of 0 with an identity placed at the location
                // of the net external wrenches in the dynamic variable vector
                iDynTree::IndexRange sensorRange =
                    this->getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR, idx);
                iDynTree::IndexRange netExtWrenchRange =
                    this->getRangeLinkVariable(NET_EXT_WRENCH, idx);

                iDynTree::Transform measurementFrame_X_link =
                    m_link_H_externalWrenchMeasurementFrame[idx].inverse();

                matrixYElements.addSubMatrix(sensorRange.offset,
                                             netExtWrenchRange.offset,
                                             measurementFrame_X_link.asAdjointTransformWrench());

                // bY for the net external wrenches is zero
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT WRENCHES
    ////////////////////////////////////////////////////////////////////////
    for (size_t i = 0; i < this->berdySensorsInfo.wrenchSensors.size(); i++) {
        // Y for the joint wrenches is just
        // six rows of 0 with an identity placed at the location
        // of the joint wrenches in the dynamic variable vector
        iDynTree::IndexRange sensorRange = this->getRangeJointSensorVariable(
            JOINT_WRENCH_SENSOR, berdySensorsInfo.wrenchSensors[i]);
        iDynTree::IndexRange jointWrenchOffset =
            this->getRangeJointVariable(JOINT_WRENCH, berdySensorsInfo.wrenchSensors[i]);

        assert(sensorRange.size == 6);
        assert(jointWrenchOffset.size == 6);
        matrixYElements.addDiagonalMatrix(sensorRange, jointWrenchOffset, 1);

        // bY for the joint wrenches is zero
    }

    Y.setFromTriplets(matrixYElements);
    return true;
}

bool BerdyHelper::initBerdyFloatingBase()
{
    bool res = true;

    assert(m_options.includeAllNetExternalWrenchesAsDynamicVariables);

    // In the floating berdy formulation, the amount of dynamic variables is 12*nrOfLinks +
    // 6*nrOfJoints + nrOfDofs
    m_nrOfDynamicalVariables =
        12 * m_model.getNrOfLinks() + 6 * m_model.getNrOfJoints() + m_model.getNrOfDOFs();
    // The dynamics equations considered by floating berdy are the acceleration propagation for each
    // joint and the Newton-Euler equations for each link
    m_nrOfDynamicEquations = 6 * m_model.getNrOfLinks() + 6 * m_model.getNrOfJoints();

    initSensorsMeasurements();

    return res;
}

iDynTree::Model& BerdyHelper::model()
{
    return this->m_model;
}

const iDynTree::Model& BerdyHelper::model() const
{
    return this->m_model;
}

iDynTree::SensorsList& BerdyHelper::sensors()
{
    return this->m_sensors;
}

const iDynTree::SensorsList& BerdyHelper::sensors() const
{
    return this->m_sensors;
}

const iDynTree::Traversal& BerdyHelper::dynamicTraversal() const
{
    return this->m_dynamicsTraversal;
}

bool BerdyHelper::isValid() const
{
    return m_areModelAndSensorsValid;
}

size_t BerdyHelper::getNrOfDynamicVariables() const
{
    return m_nrOfDynamicalVariables;
}

size_t BerdyHelper::getNrOfDynamicEquations() const
{
    return m_nrOfDynamicEquations;
}

size_t BerdyHelper::getNrOfSensorsMeasurements() const
{
    return m_nrOfSensorsMeasurements;
}

bool BerdyHelper::resizeAndZeroBerdyMatrices(iDynTree::SparseMatrix<iDynTree::ColumnMajor>& D,
                                             iDynTree::VectorDynSize& bD,
                                             iDynTree::SparseMatrix<iDynTree::ColumnMajor>& Y,
                                             iDynTree::VectorDynSize& bY)
{
    D.resize(getNrOfDynamicEquations(), getNrOfDynamicVariables());
    bD.resize(getNrOfDynamicEquations());
    Y.resize(getNrOfSensorsMeasurements(), getNrOfDynamicVariables());
    bY.resize(getNrOfSensorsMeasurements());
    D.zero();
    bD.zero();
    Y.zero();
    bY.zero();
    return true;
}

bool BerdyHelper::resizeAndZeroBerdyMatrices(iDynTree::MatrixDynSize& D,
                                             iDynTree::VectorDynSize& bD,
                                             iDynTree::MatrixDynSize& Y,
                                             iDynTree::VectorDynSize& bY)
{
    D.resize(getNrOfDynamicEquations(), getNrOfDynamicVariables());
    bD.resize(getNrOfDynamicEquations());
    Y.resize(getNrOfSensorsMeasurements(), getNrOfDynamicVariables());
    bY.resize(getNrOfSensorsMeasurements());
    D.zero();
    bD.zero();
    Y.zero();
    bY.zero();
    return true;
}

bool BerdyHelper::updateKinematicsFromTraversalFixedBase(
    const iDynTree::JointPosDoubleArray& jointPos,
    const iDynTree::JointDOFsDoubleArray& jointVel,
    const iDynTree::Vector3& gravity)
{
    return updateKinematicsFromFixedBase(
        jointPos, jointVel, m_dynamicsTraversal.getBaseLink()->getIndex(), gravity);
}

bool BerdyHelper::updateKinematicsFromFixedBase(const iDynTree::JointPosDoubleArray& jointPos,
                                                const iDynTree::JointDOFsDoubleArray& jointVel,
                                                const iDynTree::FrameIndex& fixedFrame,
                                                const iDynTree::Vector3& gravity)
{
    m_gravity = gravity;
    m_gravity6D.setLinearVec3(gravity);
    iDynTree::AngularMotionVector3 zero3;
    zero3.zero();
    m_gravity6D.setAngularVec3(zero3);

    iDynTree::Vector3 zeroAngularVel;
    zeroAngularVel.zero();

    return updateKinematicsFromFloatingBase(jointPos, jointVel, fixedFrame, zeroAngularVel);
}

bool BerdyHelper::updateKinematicsFromFloatingBase(const iDynTree::JointPosDoubleArray& jointPos,
                                                   const iDynTree::JointDOFsDoubleArray& jointVel,
                                                   const iDynTree::FrameIndex& floatingFrame,
                                                   const iDynTree::Vector3& angularVel)
{
    if (!m_areModelAndSensorsValid) {
        iDynTree::reportError("BerdyHelpers",
                              "updateKinematicsFromFloatingBase",
                              "Model and sensors information not setted.");
        return false;
    }

    if (floatingFrame == iDynTree::FRAME_INVALID_INDEX || floatingFrame < 0
        || floatingFrame >= static_cast<iDynTree::FrameIndex>(m_model.getNrOfFrames())) {
        iDynTree::reportError(
            "BerdyHelpers", "updateKinematicsFromFloatingBase", "Unknown frame index specified.");
        return false;
    }

    // Get link of the specified frame
    iDynTree::LinkIndex floatingLinkIndex = m_model.getFrameLink(floatingFrame);

    // To initialize the kinematic propagation, we should first convert the kinematics
    // information from the frame in which they are specified to the main frame of the link
    iDynTree::Transform link_H_frame = m_model.getFrameTransform(floatingFrame);

    // Convert the twist from the additional  frame to the link frame
    iDynTree::Twist base_vel_frame, base_vel_link;
    iDynTree::Vector3 zero3;
    zero3.zero();
    base_vel_frame.setLinearVec3(zero3);
    base_vel_frame.setAngularVec3(angularVel);
    base_vel_link = link_H_frame * base_vel_frame;

    // Propagate the kinematics information
    bool ok = iDynTree::dynamicsEstimationForwardVelKinematics(
        m_model,
        m_kinematicTraversals.getTraversalWithLinkAsBase(m_model, floatingLinkIndex),
        base_vel_link.getAngularVec3(),
        jointPos,
        jointVel,
        m_linkVels);

    // The jointPos and joint vel are stored directly, as are then passed to the Model object to get
    // adjacent links transforms
    m_jointPos = jointPos;
    m_jointVel = jointVel;

    m_kinematicsUpdated = ok;
    return ok;
}

bool BerdyHelper::getBerdyMatrices(iDynTree::SparseMatrix<iDynTree::ColumnMajor>& D,
                                   iDynTree::VectorDynSize& bD,
                                   iDynTree::SparseMatrix<iDynTree::ColumnMajor>& Y,
                                   iDynTree::VectorDynSize& bY)
{
    if (!m_kinematicsUpdated) {
        iDynTree::reportError("BerdyHelpers", "getBerdyMatrices", "Kinematic information not set.");
        return false;
    }

    bool res = true;

    // Compute D matrix of dynamics equations
    if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        res = res && computeBerdyDynamicsMatricesFixedBase(D, bD);
    }
    else {
        assert(m_options.berdyVariant == BERDY_FLOATING_BASE);
        res = res && computeBerdyDynamicsMatricesFloatingBase(D, bD);
    }

    // Compute Y matrix of sensors
    res = res && computeBerdySensorMatrices(Y, bY);

    return res;
}

bool BerdyHelper::getBerdyMatrices(iDynTree::MatrixDynSize& D,
                                   iDynTree::VectorDynSize& bD,
                                   iDynTree::MatrixDynSize& Y,
                                   iDynTree::VectorDynSize& bY)
{
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> DSparse(getNrOfDynamicEquations(),
                                                          getNrOfDynamicVariables());
    iDynTree::SparseMatrix<iDynTree::ColumnMajor> YSparse(getNrOfSensorsMeasurements(),
                                                          getNrOfDynamicVariables());

    bool result = getBerdyMatrices(DSparse, bD, YSparse, bY);
    if (!result)
        return false;

    for (iDynTree::SparseMatrix<iDynTree::ColumnMajor>::const_iterator it(DSparse.begin());
         it != DSparse.end();
         ++it) {
        D(it->row, it->column) = it->value;
    }

    for (iDynTree::SparseMatrix<iDynTree::ColumnMajor>::const_iterator it(YSparse.begin());
         it != YSparse.end();
         ++it) {
        Y(it->row, it->column) = it->value;
    }
    return true;
}

void BerdyHelper::cacheSensorsOrdering()
{
    // TODO: reserve space
    unsigned size = 0;
    m_sensorsOrdering.clear();
    m_sensorsOrdering.reserve(size);

    // To be a bit more flexible, rely on getRangeSensorVariable to have the order of
    // the URDF sensors
    //???: Isn't this a loop in some way??
    for (iDynTree::SensorsList::Iterator it = m_sensors.allSensorsIterator(); it.isValid(); ++it) {
        iDynTree::IndexRange sensorRange = this->getRangeSensorVariable(
            (*it)->getSensorType(),
            m_sensors.getSensorIndex((*it)->getSensorType(), (*it)->getName()));

        BerdySensor sensor;
        sensor.type = static_cast<BerdySensorTypes>((*it)->getSensorType());
        sensor.id = (*it)->getName();
        sensor.range = sensorRange;
        m_sensorsOrdering.push_back(sensor);
    }

    // For each URDF sensor, get the offset with the function
    // put everything in a sorted list (sort with the offset)
    // the iterate and add the sensors to the vector

    // The remaining sensor order is hardcoded for now
    if (m_options.includeAllJointAccelerationsAsSensors) {
        for (iDynTree::DOFIndex idx = 0;
             idx < static_cast<iDynTree::DOFIndex>(m_model.getNrOfDOFs());
             idx++) {
            iDynTree::IndexRange sensorRange =
                this->getRangeDOFSensorVariable(DOF_ACCELERATION_SENSOR, idx);
            BerdySensor jointAcc;
            jointAcc.type = DOF_ACCELERATION_SENSOR;
            jointAcc.id = m_model.getJointName(idx);
            jointAcc.range = sensorRange;
            m_sensorsOrdering.push_back(jointAcc);
        }
    }

    if (m_options.includeAllJointTorquesAsSensors) {
        for (iDynTree::DOFIndex idx = 0;
             idx < static_cast<iDynTree::DOFIndex>(m_model.getNrOfDOFs());
             idx++) {
            iDynTree::IndexRange sensorRange =
                this->getRangeDOFSensorVariable(DOF_TORQUE_SENSOR, idx);
            BerdySensor jointSens;
            jointSens.type = DOF_TORQUE_SENSOR;
            jointSens.id = m_model.getJointName(idx);
            jointSens.range = sensorRange;
            m_sensorsOrdering.push_back(jointSens);
        }
    }

    if (m_options.includeAllNetExternalWrenchesAsSensors) {
        for (iDynTree::LinkIndex idx = 0;
             idx < static_cast<iDynTree::DOFIndex>(m_model.getNrOfLinks());
             idx++) {
            // If this link is the (fixed) base link and the
            // berdy variant is ORIGINAL_BERDY_FIXED_BASE , then
            // the net wrench applied on the base is not part of the dynamical
            // system. Anyhow, we can still write the base wrench as a function
            // of sum of the joint wrenches of all the joints attached to the base (tipically just
            // one)
            if (m_dynamicsTraversal.getBaseLink()->getIndex() == idx
                && (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE
                    && !m_options.includeFixedBaseExternalWrench)) {
                continue;
            }
            iDynTree::IndexRange sensorRange =
                this->getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR, idx);
            BerdySensor linkSens;
            linkSens.type = NET_EXT_WRENCH_SENSOR;
            linkSens.id = m_model.getLinkName(idx);
            linkSens.range = sensorRange;
            m_sensorsOrdering.push_back(linkSens);
        }
    }

    for (size_t i = 0; i < this->berdySensorsInfo.wrenchSensors.size(); i++) {
        iDynTree::IndexRange sensorRange = this->getRangeJointSensorVariable(
            JOINT_WRENCH_SENSOR, berdySensorsInfo.wrenchSensors[i]);

        BerdySensor jointSens;
        jointSens.type = JOINT_WRENCH_SENSOR;
        jointSens.id = m_model.getJointName(i);
        jointSens.range = sensorRange;
        m_sensorsOrdering.push_back(jointSens);
    }

    // To avoid any problem, sort m_sensorsOrdering by range.offset
    std::sort(m_sensorsOrdering.begin(), m_sensorsOrdering.end());
}

void BerdyHelper::cacheDynamicVariablesOrderingFixedBase()
{
    m_dynamicVariablesOrdering.clear();
    unsigned size = 0;
    m_dynamicVariablesOrdering.reserve(size);

    /* Ordering:
     * For each Link - {Base} (and parente Joint) add:
     * - proper acceleration
     * - Wrench on link without gravity
     * - Joint wrench
     * - Joint torque
     * - net external wrench
     * - Joint acceleration
     */

    for (iDynTree::TraversalIndex link = 1;
         link < static_cast<iDynTree::TraversalIndex>(m_dynamicsTraversal.getNrOfVisitedLinks());
         ++link) {
        // is the following correct???
        iDynTree::LinkIndex realLinkIndex = m_dynamicsTraversal.getLink(link)->getIndex();
        const iDynTree::IJoint* joint = m_dynamicsTraversal.getParentJoint(link);
        iDynTree::JointIndex jointIndex = joint->getIndex();

        std::string linkName = m_model.getLinkName(realLinkIndex);
        std::string parentJointName = m_model.getJointName(jointIndex);

        BerdyDynamicVariable acceleration;
        acceleration.type = LINK_BODY_PROPER_ACCELERATION;
        acceleration.id = linkName;
        //???: I don't know if the following is a sort of loop.
        // Ignore for now, but in case we want to locate all the hardcoded ordering in
        // one function we have to be sure of this.
        acceleration.range = getRangeLinkVariable(acceleration.type, realLinkIndex);

        BerdyDynamicVariable linkWrench;
        linkWrench.type = NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV;
        linkWrench.id = linkName;
        linkWrench.range = getRangeLinkVariable(linkWrench.type, realLinkIndex);

        BerdyDynamicVariable jointWrench;
        jointWrench.type = JOINT_WRENCH;
        jointWrench.id = parentJointName;
        jointWrench.range = getRangeJointVariable(jointWrench.type, jointIndex);

        BerdyDynamicVariable jointTorque;
        jointTorque.type = DOF_TORQUE;
        jointTorque.id = parentJointName;
        // TODO: for now assume 1 dof joint
        jointTorque.range = getRangeDOFVariable(jointTorque.type, jointIndex);

        BerdyDynamicVariable netExternalWrench;
        netExternalWrench.type = NET_EXT_WRENCH;
        netExternalWrench.id = linkName;
        netExternalWrench.range = getRangeLinkVariable(netExternalWrench.type, realLinkIndex);

        BerdyDynamicVariable jointAcceleration;
        jointAcceleration.type = DOF_ACCELERATION;
        jointAcceleration.id = parentJointName;
        // TODO: for now assume 1 dof joint
        jointAcceleration.range = getRangeDOFVariable(jointAcceleration.type, jointIndex);

        m_dynamicVariablesOrdering.push_back(acceleration);
        m_dynamicVariablesOrdering.push_back(linkWrench);
        m_dynamicVariablesOrdering.push_back(jointWrench);
        m_dynamicVariablesOrdering.push_back(jointTorque);
        m_dynamicVariablesOrdering.push_back(netExternalWrench);
        m_dynamicVariablesOrdering.push_back(jointAcceleration);
    }

    // To avoid any problem, sort m_dynamicVariablesOrdering by range.offset
    std::sort(m_dynamicVariablesOrdering.begin(), m_dynamicVariablesOrdering.end());
}

void BerdyHelper::cacheDynamicVariablesOrderingFloatingBase()
{
    m_dynamicVariablesOrdering.clear();
    unsigned size = 0;
    m_dynamicVariablesOrdering.reserve(size);

    /*
     * Ordering:
     * The serialization we use is:
     * * All the link variables (proper classical acc and external force-torque), ordered using the
     * link index.
     * * All the joint variables (joint force-torque), ordered using the joint index.
     * * All the dof variables (dof acceleration), ordered using the dof index.
     */

    for (iDynTree::LinkIndex link = 0;
         link < static_cast<iDynTree::LinkIndex>(m_model.getNrOfLinks());
         ++link) {
        std::string linkName = m_model.getLinkName(link);

        BerdyDynamicVariable acceleration;
        acceleration.type = LINK_BODY_PROPER_CLASSICAL_ACCELERATION;
        acceleration.id = linkName;
        acceleration.range = getRangeLinkVariable(acceleration.type, link);

        BerdyDynamicVariable netExtWrench;
        netExtWrench.type = NET_EXT_WRENCH;
        netExtWrench.id = linkName;
        netExtWrench.range = getRangeLinkVariable(netExtWrench.type, link);

        m_dynamicVariablesOrdering.push_back(acceleration);
        m_dynamicVariablesOrdering.push_back(netExtWrench);
    }

    for (iDynTree::JointIndex jntIdx = 0;
         jntIdx < static_cast<iDynTree::JointIndex>(m_model.getNrOfJoints());
         ++jntIdx) {
        iDynTree::IJointPtr joint = m_model.getJoint(jntIdx);

        BerdyDynamicVariable jointForceTorque;
        jointForceTorque.type = JOINT_WRENCH;
        jointForceTorque.id = m_model.getJointName(jntIdx);
        jointForceTorque.range = getRangeJointVariable(jointForceTorque.type, jntIdx);

        m_dynamicVariablesOrdering.push_back(jointForceTorque);

        // If the joint is not fixed, we also add the descriptor of the acceleration of the relative
        // dof The internal order of the descriptors will be fixed by the call to std::sort
        if (joint->getNrOfDOFs() > 0) {
            // TODO(traversaro) At the time of implementing this (late 2017) the concept of dof name
            // is not present in iDynTree . We will then just assume that the joint have at maximum
            // one dof. This can be fixed in the future by introducing a proper dof name
            assert(joint->getNrOfDOFs() == 1);

            BerdyDynamicVariable dofAcceleration;
            dofAcceleration.type = DOF_ACCELERATION;
            dofAcceleration.id = m_model.getJointName(jntIdx);
            dofAcceleration.range =
                getRangeDOFVariable(dofAcceleration.type, joint->getDOFsOffset());

            m_dynamicVariablesOrdering.push_back(dofAcceleration);
        }
    }

    // To avoid any problem, sort m_dynamicVariablesOrdering by range.offset
    std::sort(m_dynamicVariablesOrdering.begin(), m_dynamicVariablesOrdering.end());
}

const std::vector<BerdySensor>& BerdyHelper::getSensorsOrdering() const
{
    return m_sensorsOrdering;
}

const std::vector<BerdyDynamicVariable>& BerdyHelper::getDynamicVariablesOrdering() const
{
    return m_dynamicVariablesOrdering;
}

bool BerdyHelper::serializeDynamicVariables(
    iDynTree::LinkProperAccArray& properAccs,
    iDynTree::LinkNetTotalWrenchesWithoutGravity& netTotalWrenchesWithoutGrav,
    iDynTree::LinkNetExternalWrenches& netExtWrenches,
    iDynTree::LinkInternalWrenches& linkJointWrenches,
    iDynTree::JointDOFsDoubleArray& jointTorques,
    iDynTree::JointDOFsDoubleArray& jointAccs,
    iDynTree::VectorDynSize& d)
{
    bool res = false;
    switch (m_options.berdyVariant) {
        case ORIGINAL_BERDY_FIXED_BASE:
            res = serializeDynamicVariablesFixedBase(properAccs,
                                                     netTotalWrenchesWithoutGrav,
                                                     netExtWrenches,
                                                     linkJointWrenches,
                                                     jointTorques,
                                                     jointAccs,
                                                     d);
            break;

        case BERDY_FLOATING_BASE:
            res = serializeDynamicVariablesFloatingBase(properAccs,
                                                        netTotalWrenchesWithoutGrav,
                                                        netExtWrenches,
                                                        linkJointWrenches,
                                                        jointTorques,
                                                        jointAccs,
                                                        d);
            break;

        default:
            iDynTree::reportError(
                "BerdyHelpers", "serializeDynamicVariablesFixedBase", "unknown berdy variant");
            assert(false);
            res = false;
    }
    return res;
}

bool BerdyHelper::serializeDynamicVariablesFixedBase(
    iDynTree::LinkProperAccArray& properAccs,
    iDynTree::LinkNetTotalWrenchesWithoutGravity& netTotalWrenchesWithoutGrav,
    iDynTree::LinkNetExternalWrenches& netExtWrenches,
    iDynTree::LinkInternalWrenches& linkJointWrenches,
    iDynTree::JointDOFsDoubleArray& jointTorques,
    iDynTree::JointDOFsDoubleArray& jointAccs,
    iDynTree::VectorDynSize& d)
{
    assert(d.size() == this->getNrOfDynamicVariables());
    assert(this->m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE);

    if (this->m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE) {
        // In the original formulation, also the totalNetWrenchesWithoutGravity where part of the
        // variables And the base link is not considered amoing the dynamic variables
        for (iDynTree::LinkIndex linkIdx = 0;
             linkIdx < static_cast<iDynTree::LinkIndex>(m_model.getNrOfLinks());
             linkIdx++) {
            // The base variables are not estimated for ORIGINAL_BERDY_FIXED_BASE
            if (linkIdx != this->m_dynamicsTraversal.getBaseLink()->getIndex()) {
                setSubVector(d,
                             getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION, linkIdx),
                             toEigen(properAccs(linkIdx)));
                setSubVector(
                    d,
                    getRangeLinkVariable(NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV, linkIdx),
                    toEigen(netTotalWrenchesWithoutGrav(linkIdx)));
                if (m_options.includeAllNetExternalWrenchesAsDynamicVariables) {
                    setSubVector(d,
                                 getRangeLinkVariable(NET_EXT_WRENCH, linkIdx),
                                 toEigen(netExtWrenches(linkIdx)));
                }
            }
        }

        for (iDynTree::JointIndex jntIdx = 0;
             jntIdx < static_cast<iDynTree::JointIndex>(m_model.getNrOfJoints());
             jntIdx++) {
            iDynTree::IJointConstPtr jnt = m_model.getJoint(jntIdx);

            iDynTree::LinkIndex childLink =
                m_dynamicsTraversal.getChildLinkIndexFromJointIndex(m_model, jntIdx);
            setSubVector(d,
                         getRangeJointVariable(JOINT_WRENCH, jntIdx),
                         iDynTree::toEigen(linkJointWrenches(childLink)));

            // For ORIGINAL_BERDY_FIXED_BASE, we know that every joint has 1 dof
            setSubVector(
                d, getRangeDOFVariable(DOF_ACCELERATION, jntIdx), jointAccs(jnt->getDOFsOffset()));
            setSubVector(
                d, getRangeDOFVariable(DOF_TORQUE, jntIdx), jointTorques(jnt->getDOFsOffset()));
        }
    }

    return true;
}

bool BerdyHelper::serializeDynamicVariablesFloatingBase(
    iDynTree::LinkProperAccArray& properAccs,
    iDynTree::LinkNetTotalWrenchesWithoutGravity& /*netTotalWrenchesWithoutGrav*/,
    iDynTree::LinkNetExternalWrenches& netExtWrenches,
    iDynTree::LinkInternalWrenches& linkJointWrenches,
    iDynTree::JointDOFsDoubleArray&,
    iDynTree::JointDOFsDoubleArray& jointAccs,
    iDynTree::VectorDynSize& d)
{
    d.resize(this->getNrOfDynamicVariables());
    assert(this->m_options.berdyVariant == BERDY_FLOATING_BASE);

    for (iDynTree::LinkIndex link = 0;
         link < static_cast<iDynTree::LinkIndex>(m_model.getNrOfLinks());
         ++link) {
        // Handle acceleration
        // Convert left trivialized proper acceleration to sensor proper acceleration
        iDynTree::ClassicalAcc sensorProperAcc;

        sensorProperAcc.fromSpatial(properAccs(link), m_linkVels(link));
        setSubVector(d,
                     getRangeLinkVariable(LINK_BODY_PROPER_CLASSICAL_ACCELERATION, link),
                     toEigen(sensorProperAcc));

        // Handle external force
        setSubVector(d, getRangeLinkVariable(NET_EXT_WRENCH, link), toEigen(netExtWrenches(link)));
    }

    for (iDynTree::JointIndex jntIdx = 0;
         jntIdx < static_cast<iDynTree::JointIndex>(m_model.getNrOfJoints());
         ++jntIdx) {
        iDynTree::IJointPtr joint = m_model.getJoint(jntIdx);

        // Handle joint wrench
        // Warning: for legacy reason linkJointWrenches is addressed using the child link index of
        // its joint
        iDynTree::LinkIndex childLinkIndex =
            m_dynamicsTraversal.getChildLinkIndexFromJointIndex(m_model, jntIdx);
        setSubVector(d,
                     getRangeJointVariable(JOINT_WRENCH, jntIdx),
                     toEigen(linkJointWrenches(childLinkIndex)));

        // If the joint is not fixed, we also need to serialize the acceleration of the relative dof
        if (joint->getNrOfDOFs() > 0) {
            for (unsigned int localDof = 0; localDof < joint->getNrOfDOFs(); localDof++) {
                iDynTree::DOFIndex dofIdx = joint->getDOFsOffset() + localDof;
                setSubVector(d, getRangeDOFVariable(DOF_ACCELERATION, dofIdx), jointAccs(dofIdx));
            }
        }
    }

    return true;
}

bool BerdyHelper::serializeDynamicVariablesComputedFromFixedBaseRNEA(
    iDynTree::JointDOFsDoubleArray& jointAccs,
    iDynTree::LinkNetExternalWrenches& netExtWrenches,
    iDynTree::VectorDynSize& d)
{
    assert(jointAccs.size() == this->m_model.getNrOfDOFs());
    assert(netExtWrenches.isConsistent(this->m_model));

    iDynTree::LinkInternalWrenches intWrenches(this->m_model);
    iDynTree::FreeFloatingGeneralizedTorques genTrqs(this->m_model);

    iDynTree::LinkVelArray linkVels(this->m_model);
    iDynTree::LinkAccArray linkProperAccs(this->m_model);

    iDynTree::Vector3 baseProperAcc;
    iDynTree::toEigen(baseProperAcc) = -toEigen(m_gravity);
    iDynTree::Vector3 zeroVec;
    zeroVec.zero();
    iDynTree::dynamicsEstimationForwardVelAccKinematics(m_model,
                                                        m_dynamicsTraversal,
                                                        baseProperAcc,
                                                        zeroVec,
                                                        zeroVec,
                                                        m_jointPos,
                                                        m_jointVel,
                                                        jointAccs,
                                                        linkVels,
                                                        linkProperAccs);

    iDynTree::RNEADynamicPhase(m_model,
                               m_dynamicsTraversal,
                               m_jointPos,
                               linkVels,
                               linkProperAccs,
                               netExtWrenches,
                               intWrenches,
                               genTrqs);

    // Generate the d vector of dynamical variables
    assert(d.size() == this->getNrOfDynamicVariables());

    // LinkNewInternalWrenches (necessary for the old-style berdy)
    iDynTree::LinkNetTotalWrenchesWithoutGravity linkNetWrenchesWithoutGravity(this->model());

    for (iDynTree::LinkIndex visitedLinkIndex = 0;
         visitedLinkIndex < static_cast<iDynTree::LinkIndex>(model().getNrOfLinks());
         visitedLinkIndex++) {
        iDynTree::LinkConstPtr visitedLink = this->model().getLink(visitedLinkIndex);

        const iDynTree::SpatialInertia& I = visitedLink->getInertia();
        const iDynTree::SpatialAcc& properAcc = linkProperAccs(visitedLinkIndex);
        const iDynTree::Twist& v = linkVels(visitedLinkIndex);
        linkNetWrenchesWithoutGravity(visitedLinkIndex) = I * properAcc + v * (I * v);
    }

    return serializeDynamicVariables(linkProperAccs,
                                     linkNetWrenchesWithoutGravity,
                                     netExtWrenches,
                                     intWrenches,
                                     genTrqs.jointTorques(),
                                     jointAccs,
                                     d);
}

bool BerdyHelper::serializeSensorVariables(iDynTree::SensorsMeasurements& sensMeas,
                                           iDynTree::LinkNetExternalWrenches& netExtWrenches,
                                           iDynTree::JointDOFsDoubleArray& jointTorques,
                                           iDynTree::JointDOFsDoubleArray& jointAccs,
                                           iDynTree::LinkInternalWrenches& linkJointWrenches,
                                           iDynTree::VectorDynSize& y)
{
    bool ret = true;
    assert(y.size() == this->getNrOfSensorsMeasurements());

    bool ok = sensMeas.toVector(realSensorMeas);
    ret = ret && ok;

    // The first part of the sensor vector is exactly the serialization of the sensMeas object
    iDynTree::IndexRange ran;
    ran.offset = 0;
    ran.size = realSensorMeas.size();
    setSubVector(y, ran, toEigen(realSensorMeas));

    // Then we need to follow the serialization used in computeBerdySensorMatrices

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT ACCELERATIONS
    ////////////////////////////////////////////////////////////////////////
    if (m_options.includeAllJointAccelerationsAsSensors) {
        for (iDynTree::DOFIndex idx = 0;
             idx < static_cast<iDynTree::DOFIndex>(m_model.getNrOfDOFs());
             idx++) {
            iDynTree::IndexRange sensorRange =
                this->getRangeDOFSensorVariable(DOF_ACCELERATION_SENSOR, idx);

            setSubVector(y, sensorRange, jointAccs(idx));
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT TORQUES
    ////////////////////////////////////////////////////////////////////////
    if (m_options.includeAllJointTorquesAsSensors) {
        for (iDynTree::DOFIndex idx = 0;
             idx < static_cast<iDynTree::DOFIndex>(m_model.getNrOfDOFs());
             idx++) {
            iDynTree::IndexRange sensorRange =
                this->getRangeDOFSensorVariable(DOF_TORQUE_SENSOR, idx);

            setSubVector(y, sensorRange, jointTorques(idx));
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// NET EXTERNAL WRENCHES ACTING ON LINKS
    ////////////////////////////////////////////////////////////////////////
    if (m_options.includeAllNetExternalWrenchesAsSensors) {
        for (iDynTree::LinkIndex idx = 0;
             idx < static_cast<iDynTree::LinkIndex>(m_model.getNrOfLinks());
             idx++) {
            if (!(m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE
                  && m_dynamicsTraversal.getBaseLink()->getIndex() == idx)
                || m_options.includeFixedBaseExternalWrench) {
                iDynTree::IndexRange sensorRange =
                    this->getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR, idx);

                setSubVector(y,
                             sensorRange,
                             toEigen(m_link_H_externalWrenchMeasurementFrame[idx].inverse()
                                     * netExtWrenches(idx)));
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT WRENCHES
    ////////////////////////////////////////////////////////////////////////
    for (size_t i = 0; i < this->berdySensorsInfo.wrenchSensors.size(); i++) {
        iDynTree::IndexRange sensorRange = this->getRangeJointSensorVariable(
            JOINT_WRENCH_SENSOR, berdySensorsInfo.wrenchSensors[i]);

        iDynTree::LinkIndex childLink = m_dynamicsTraversal.getChildLinkIndexFromJointIndex(
            m_model, berdySensorsInfo.wrenchSensors[i]);
        setSubVector(y, sensorRange, toEigen(linkJointWrenches(childLink)));
    }

    return ret;
}

bool BerdyHelper::extractJointTorquesFromDynamicVariables(
    const iDynTree::VectorDynSize& d,
    const iDynTree::VectorDynSize& jointPos,
    iDynTree::VectorDynSize& jointTorques) const
{
    const iDynTree::Model& model = this->model();

    for (iDynTree::JointIndex jntIdx = 0;
         jntIdx < static_cast<iDynTree::JointIndex>(model.getNrOfJoints());
         ++jntIdx) {
        iDynTree::IJointConstPtr joint = model.getJoint(jntIdx);

        // Handle joint wrench
        // Warning: for legacy reason linkJointWrenches is addressed using the child link index of
        // its joint
        iDynTree::LinkIndex childLinkIndex =
            this->dynamicTraversal().getChildLinkIndexFromJointIndex(model, jntIdx);
        iDynTree::LinkIndex parentLinkIndex =
            this->dynamicTraversal().getParentLinkIndexFromJointIndex(model, jntIdx);

        iDynTree::IndexRange range = this->getRangeJointVariable(JOINT_WRENCH, jntIdx);
        iDynTree::LinearForceVector3 force(d.data() + range.offset, 3);
        iDynTree::AngularMotionVector3 torque(d.data() + range.offset + 3, 3);
        iDynTree::Wrench jointWrench = iDynTree::Wrench(force, torque);

        // If the joint is not fixed, we can compute the joint torque
        joint->computeJointTorque(
            jointPos, jointWrench, parentLinkIndex, childLinkIndex, jointTorques);
    }

    return true;
}

bool BerdyHelper::extractLinkNetExternalWrenchesFromDynamicVariables(
    const iDynTree::VectorDynSize& d,
    iDynTree::LinkNetExternalWrenches& netExtWrenches) const
{
    const iDynTree::Model& model = this->model();
    for (iDynTree::LinkIndex lnkIdx = 0;
         lnkIdx < static_cast<iDynTree::LinkIndex>(model.getNrOfLinks());
         lnkIdx++) {
        iDynTree::IndexRange range = this->getRangeLinkVariable(NET_EXT_WRENCH, lnkIdx);
        iDynTree::LinearForceVector3 force(d.data() + range.offset, 3);
        iDynTree::AngularMotionVector3 torque(d.data() + range.offset + 3, 3);
        netExtWrenches(lnkIdx) = iDynTree::Wrench(force, torque);
    }

    return true;
}

bool BerdyHelper::setNetExternalWrenchMeasurementFrame(
    const iDynTree::LinkIndex lnkIndex,
    const iDynTree::Transform& link_H_externalWrenchMeasurementFrame)
{
    if (!m_model.isValidLinkIndex(lnkIndex))
        return false;

    m_link_H_externalWrenchMeasurementFrame[lnkIndex] = link_H_externalWrenchMeasurementFrame;

    return true;
}

bool BerdyHelper::getNetExternalWrenchMeasurementFrame(
    const iDynTree::LinkIndex lnkIndex,
    iDynTree::Transform& link_H_externalWrenchMeasurementFrame) const
{
    if (!m_model.isValidLinkIndex(lnkIndex))
        return false;

    link_H_externalWrenchMeasurementFrame = m_link_H_externalWrenchMeasurementFrame[lnkIndex];

    return true;
}
