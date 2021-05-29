function inverseDynamicsJointTorques = computeIDJointTorques(Data, modelPath, modelFileName, jointOrder)

    timeExt = Data.data.time'/1e9;
    jointPos = Data.data.jointPositions;
    jointVel = Data.data.jointVelocities;
    jointAcc = Data.data.jointAccs;


    n_joints = size(jointPos, 1);
    
    KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,'Pelvis',modelPath,modelFileName,false);
    
    
    KinDynModel.kinDynComp.setFrameVelocityRepresentation(iDynTree.MIXED_REPRESENTATION);
    
    rightfootIdx = KinDynModel.kinDynComp.model().getLinkIndex('RightFoot');
    leftfootIdx = KinDynModel.kinDynComp.model().getLinkIndex('LeftFoot');
    
    
    gravity = iDynTree.Vector3();
    gravity.fromMatlab([0, 0, -9.8]);
    
    
    gravityTrqs = zeros(size(Data.data.jointTorques));
    inertialTrqs = zeros(size(Data.data.jointTorques));
    inverseDynamicsJointTorques = zeros(size(Data.data.jointTorques));
    
    w_H_b = iDynTree.Matrix4x4;
    w_H_b.zero();
    
    baseTF = iDynTree.Transform();
    
    baseVel = iDynTree.Twist();
    baseVel.zero();
    
    baseAcc = iDynTree.Vector6();
    baseAcc.zero();
    
    q = iDynTree.JointPosDoubleArray(n_joints);
    dq = iDynTree.VectorDynSize(n_joints);
    d2q = iDynTree.VectorDynSize(n_joints);
    

    for t = 1 : length(timeExt)
        
        
        q.fromMatlab((jointPos(:, t)));

        dq.fromMatlab((jointVel(:, t)));
        
        d2q.fromMatlab((jointAcc(:, t)));
        
        baseVel.setVal(0, Data.linkData(1).data.velocity(1, t));
        baseVel.setVal(1, Data.linkData(1).data.velocity(2, t));
        baseVel.setVal(2, Data.linkData(1).data.velocity(3, t));
        baseVel.setVal(3, Data.linkData(1).data.velocity(4, t));
        baseVel.setVal(4, Data.linkData(1).data.velocity(5, t));
        baseVel.setVal(5, Data.linkData(1).data.velocity(6, t));
        
        
        basePose = Data.linkData(1).data.pose;
        baseRot = iDynTree.Rotation.RPY(basePose(4), basePose(5), basePose(6));
        
        baseTF.setPosition(iDynTree.Position(basePose(1), basePose(2), basePose(3)));
        baseTF.setRotation(baseRot);
        
        KinDynModel.kinDynComp.setRobotState(baseTF, q, baseVel, dq, gravity);
        
        generalizedTrqs = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model());
        KinDynModel.kinDynComp.generalizedGravityForces(generalizedTrqs);
        
        gravityTrqs(:, t) = generalizedTrqs.jointTorques.toMatlab;
        
        generalizedbiasTrqs =  iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model());
        KinDynModel.kinDynComp.generalizedBiasForces(generalizedbiasTrqs);
        
        biasTrqs(:, t) = generalizedbiasTrqs.jointTorques.toMatlab;
        
        baseAcc.setVal(0, Data.linkData(1).data.acceleration(1, t));
        baseAcc.setVal(1, Data.linkData(1).data.acceleration(2, t));
        baseAcc.setVal(2, Data.linkData(1).data.acceleration(3, t));
        baseAcc.setVal(3, Data.linkData(1).data.acceleration(4, t));
        baseAcc.setVal(4, Data.linkData(1).data.acceleration(5, t));
        baseAcc.setVal(5, Data.linkData(1).data.acceleration(6, t));
        
        
        %% Get link transform
        w_rpy_lefttFoot = Data.linkData(14).data.pose(4:6, t);
        w_rpy_rightFoot = Data.linkData(3).data.pose(4:6, t);
        
        w_R_leftFoot = iDynTree.Rotation.RPY(w_rpy_lefttFoot(1), w_rpy_lefttFoot(2), w_rpy_lefttFoot(3));
        w_R_rightFoot = iDynTree.Rotation.RPY(w_rpy_rightFoot(1), w_rpy_rightFoot(2), w_rpy_rightFoot(3));
        
        linkNetWrench = iDynTree.LinkWrenches(KinDynModel.kinDynComp.model());
        linkNetWrench.zero();
        
        leftContactWrench = linkNetWrench(leftfootIdx);
        rightContactWrench = linkNetWrench(rightfootIdx);
        
        leftContactWrench.setVal(0, Data.data.task2_wrenchMeasurementsInLinkFrame(1, t));
        leftContactWrench.setVal(1, Data.data.task2_wrenchMeasurementsInLinkFrame(2, t));
        leftContactWrench.setVal(2, Data.data.task2_wrenchMeasurementsInLinkFrame(3, t));
        leftContactWrench.setVal(3, Data.data.task2_wrenchMeasurementsInLinkFrame(4, t));
        leftContactWrench.setVal(4, Data.data.task2_wrenchMeasurementsInLinkFrame(5, t));
        leftContactWrench.setVal(5, Data.data.task2_wrenchMeasurementsInLinkFrame(6, t));
        
        rightContactWrench.setVal(0, Data.data.task2_wrenchMeasurementsInLinkFrame(7, t));
        rightContactWrench.setVal(1, Data.data.task2_wrenchMeasurementsInLinkFrame(8, t));
        rightContactWrench.setVal(2, Data.data.task2_wrenchMeasurementsInLinkFrame(9, t));
        rightContactWrench.setVal(3, Data.data.task2_wrenchMeasurementsInLinkFrame(10, t));
        rightContactWrench.setVal(4, Data.data.task2_wrenchMeasurementsInLinkFrame(11, t));
        rightContactWrench.setVal(5, Data.data.task2_wrenchMeasurementsInLinkFrame(12, t));
        
        leftContactWrench = w_R_leftFoot * leftContactWrench;
        rightContactWrench = w_R_rightFoot * rightContactWrench;
        
        baseForceAndJointTorques = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model());
        KinDynModel.kinDynComp.inverseDynamics(baseAcc, d2q, linkNetWrench, baseForceAndJointTorques);
        inverseDynamicsJointTorques(:, t) = baseForceAndJointTorques.jointTorques.toMatlab;
        
        inertialTrqs(:, t) = inverseDynamicsJointTorques(:, t) - gravityTrqs(:, t) - biasTrqs(:, t);
        
    end
    
end