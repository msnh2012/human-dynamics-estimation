#!/bin/sh

echo "removeWrenchOffset" | yarp rpc /Human-without-sot-1e-6/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-without-sot-1e-4/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-without-sot-1e-3/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-without-sot-1e-2/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-without-sot-1e-1/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-without-sot-1e0/HumanDynamicsEstimator/rpc:i


echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e-4/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e-3/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e-2/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e-1/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e0/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e1/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e2/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e3/HumanDynamicsEstimator/rpc:i
echo "removeWrenchOffset" | yarp rpc /Human-with-sot-1e4/HumanDynamicsEstimator/rpc:i
