#!/bin/sh

echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e-4/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e0/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e4/HumanDynamicsEstimator/rpc:i

echo "removeWrenchOffset" | yarp rpc /Human-without-sot-1e-4/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-without-sot-1e0/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-without-sot-1e4/HumanDynamicsEstimator/rpc:i
