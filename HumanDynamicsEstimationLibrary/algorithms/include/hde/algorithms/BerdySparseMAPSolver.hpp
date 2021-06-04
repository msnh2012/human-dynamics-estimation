/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef HDE_BERDY_SPARSEMAPSOLVER_H
#define HDE_BERDY_SPARSEMAPSOLVER_H

#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Model/Indices.h>
#include <iDynTree/Model/JointState.h>

namespace hde::algorithms {
    class BerdyHelper;
    class BerdySparseMAPSolver;
} // namespace hde::algorithms

/**
 * @warning This class is still in active development, and so API interface can change between
 * iDynTree versions. \ingroup iDynTreeExperimental
 */
class hde::algorithms::BerdySparseMAPSolver
{

    class BerdySparseMAPSolverPimpl;
    BerdySparseMAPSolverPimpl* m_pimpl;

public:
    BerdySparseMAPSolver(hde::algorithms::BerdyHelper& berdyHelper);
    ~BerdySparseMAPSolver();

    void setDynamicsConstraintsPriorCovariance(
        const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance);
    void setDynamicsRegularizationPriorCovariance(
        const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance);
    void setDynamicsRegularizationPriorExpectedValue(const iDynTree::VectorDynSize& expectedValue);
    void
    setMeasurementsPriorCovariance(const iDynTree::SparseMatrix<iDynTree::ColumnMajor>& covariance);

    const iDynTree::SparseMatrix<iDynTree::ColumnMajor>&
    dynamicsConstraintsPriorCovarianceInverse() const; // Sigma_D^-1
    iDynTree::SparseMatrix<iDynTree::ColumnMajor>&
    dynamicsConstraintsPriorCovarianceInverse(); // Sigma_D^-1
    const iDynTree::SparseMatrix<iDynTree::ColumnMajor>&
    dynamicsRegularizationPriorCovarianceInverse() const; // Sigma_d^-1
    iDynTree::SparseMatrix<iDynTree::ColumnMajor>&
    dynamicsRegularizationPriorCovarianceInverse(); // Sigma_d^-1
    const iDynTree::VectorDynSize& dynamicsRegularizationPriorExpectedValue() const; // mu_d
    iDynTree::VectorDynSize& dynamicsRegularizationPriorExpectedValue(); // mu_d
    const iDynTree::SparseMatrix<iDynTree::ColumnMajor>&
    measurementsPriorCovarianceInverse() const; // Sigma_y^-1
    iDynTree::SparseMatrix<iDynTree::ColumnMajor>&
    measurementsPriorCovarianceInverse(); // Sigma_y^-1

    bool isValid();

    bool initialize();

    void
    updateEstimateInformationFixedBase(const iDynTree::JointPosDoubleArray& jointsConfiguration,
                                       const iDynTree::JointDOFsDoubleArray& jointsVelocity,
                                       const iDynTree::FrameIndex fixedFrame,
                                       const iDynTree::Vector3& gravityInFixedFrame,
                                       const iDynTree::VectorDynSize& measurements);

    void updateEstimateInformationFloatingBase(
        const iDynTree::JointPosDoubleArray& jointsConfiguration,
        const iDynTree::JointDOFsDoubleArray& jointsVelocity,
        const iDynTree::FrameIndex floatingFrame,
        const iDynTree::Vector3& bodyAngularVelocityOfSpecifiedFrame,
        const iDynTree::VectorDynSize& measurements);

    bool doEstimate();

    void getLastEstimate(iDynTree::VectorDynSize& lastEstimate) const;
    const iDynTree::VectorDynSize& getLastEstimate() const;
};

#endif // HDE_BERDY_SPARSEMAPSOLVER_H
