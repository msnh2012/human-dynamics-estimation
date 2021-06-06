function [inverseDynamicsJointTorques, gravityTrqs, biasTrqs, baseWrench] = computeIDJointTorques(Data, modelPath, modelFileName, jointOrder)

    timeExt = Data.data.time'/1e9;
    jointPos = Data.data.jointPositions;
    jointVel = Data.data.jointVelocities;
    jointAcc = Data.data.jointAccs;

    n_joints = size(jointPos, 1);
    
    KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,'Pelvis',modelPath,modelFileName,false);
    
    
    KinDynModel.kinDynComp.setFrameVelocityRepresentation(iDynTree.MIXED_REPRESENTATION);
    
    rightfootIdx = KinDynModel.kinDynComp.model().getLinkIndex('RightFoot');
    leftfootIdx = KinDynModel.kinDynComp.model().getLinkIndex('LeftFoot');
    
    righthandIdx = KinDynModel.kinDynComp.model().getLinkIndex('RightHand');
    
    gravity = iDynTree.Vector3();
    gravity.fromMatlab([0, 0, -9.81]);
    
    
    gravityTrqs = zeros(size(Data.data.jointTorques));
    inertialTrqs = zeros(size(Data.data.jointTorques));
    inverseDynamicsJointTorques = zeros(size(Data.data.jointTorques));
    
    baseWrench = [];
        
    w_H_b = iDynTree.Matrix4x4;
    w_H_b.zero();
    
    baseTF = iDynTree.Transform();
    baseTF = iDynTree.Transform.Identity();
    
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
        
        leftContactWrenchLink = iDynTree.Wrench();
        leftContactWrenchLink.zero();
        
        leftContactWrenchLink.setVal(0, Data.data.task2_wrenchMeasurementsInLinkFrame(1, t));
        leftContactWrenchLink.setVal(1, Data.data.task2_wrenchMeasurementsInLinkFrame(2, t));
        leftContactWrenchLink.setVal(2, Data.data.task2_wrenchMeasurementsInLinkFrame(3, t));
        leftContactWrenchLink.setVal(3, Data.data.task2_wrenchMeasurementsInLinkFrame(4, t));
        leftContactWrenchLink.setVal(4, Data.data.task2_wrenchMeasurementsInLinkFrame(5, t));
        leftContactWrenchLink.setVal(5, Data.data.task2_wrenchMeasurementsInLinkFrame(6, t));
        
        rightContactWrenchLink = iDynTree.Wrench();
        rightContactWrenchLink.zero();
        
        rightContactWrenchLink.setVal(0, Data.data.task2_wrenchMeasurementsInLinkFrame(7, t));
        rightContactWrenchLink.setVal(1, Data.data.task2_wrenchMeasurementsInLinkFrame(8, t));
        rightContactWrenchLink.setVal(2, Data.data.task2_wrenchMeasurementsInLinkFrame(9, t));
        rightContactWrenchLink.setVal(3, Data.data.task2_wrenchMeasurementsInLinkFrame(10, t));
        rightContactWrenchLink.setVal(4, Data.data.task2_wrenchMeasurementsInLinkFrame(11, t));
        rightContactWrenchLink.setVal(5, Data.data.task2_wrenchMeasurementsInLinkFrame(12, t));
        
        leftContactWrenchLink = w_R_leftFoot * leftContactWrenchLink;
        rightContactWrenchLink = w_R_rightFoot * rightContactWrenchLink;
        
        leftContactWrench.setVal(0, leftContactWrenchLink.getVal(0));
        leftContactWrench.setVal(1, leftContactWrenchLink.getVal(1));
        leftContactWrench.setVal(2, leftContactWrenchLink.getVal(2));
        leftContactWrench.setVal(3, leftContactWrenchLink.getVal(3));
        leftContactWrench.setVal(4, leftContactWrenchLink.getVal(4));
        leftContactWrench.setVal(5, leftContactWrenchLink.getVal(5));
        
        rightContactWrench.setVal(0, rightContactWrenchLink.getVal(0));
        rightContactWrench.setVal(1, rightContactWrenchLink.getVal(1));
        rightContactWrench.setVal(2, rightContactWrenchLink.getVal(2));
        rightContactWrench.setVal(3, rightContactWrenchLink.getVal(3));
        rightContactWrench.setVal(4, rightContactWrenchLink.getVal(4));
        rightContactWrench.setVal(5, rightContactWrenchLink.getVal(5));


        rightHandContactWrench = linkNetWrench(righthandIdx);
        rightHandContactWrench.zero();
        
        hand_H_handCom = iDynTree.Transform.Identity();
        hand_H_handCom.setPosition(iDynTree.Position(0, -0.096206, 0));
        
        rightHandComContactWrench = iDynTree.Wrench;
        rightHandComContactWrench.zero();
% %         rightHandComContactWrench.setVal(2, 5 * -9.81); %%This is in inertial frame

        
        rightHandComContactWrench.setVal(0, Data.data.task2_wrenchEstimatesInLinkFrame(19, t));
        rightHandComContactWrench.setVal(1, Data.data.task2_wrenchEstimatesInLinkFrame(20, t));
        rightHandComContactWrench.setVal(2, Data.data.task2_wrenchEstimatesInLinkFrame(21, t));
        rightHandComContactWrench.setVal(3, Data.data.task2_wrenchEstimatesInLinkFrame(22, t));
        rightHandComContactWrench.setVal(4, Data.data.task2_wrenchEstimatesInLinkFrame(23, t));
        rightHandComContactWrench.setVal(5, Data.data.task2_wrenchEstimatesInLinkFrame(24, t));
        
        
        wrench = hand_H_handCom.asAdjointTransformWrench.toMatlab * rightHandComContactWrench.toMatlab;
                
        rightHandComContactWrench.setVal(0, wrench(1));
        rightHandComContactWrench.setVal(1, wrench(2));
        rightHandComContactWrench.setVal(2, wrench(3));
        rightHandComContactWrench.setVal(3, wrench(4));
        rightHandComContactWrench.setVal(4, wrench(5));
        rightHandComContactWrench.setVal(5, wrench(6));
        
               
        w_rpy_rightHand = Data.linkData(20).data.pose(4:6, t);
        w_R_rightHand = iDynTree.Rotation.RPY(w_rpy_rightHand(1), w_rpy_rightHand(2), w_rpy_rightHand(3));
        
        rightHandComContactWrench = w_R_rightHand * rightHandComContactWrench;
        
        rightHandContactWrench.setVal(0, rightHandComContactWrench.getVal(0));
        rightHandContactWrench.setVal(1, rightHandComContactWrench.getVal(1));
        rightHandContactWrench.setVal(2, rightHandComContactWrench.getVal(2));
        rightHandContactWrench.setVal(3, rightHandComContactWrench.getVal(3));
        rightHandContactWrench.setVal(4, rightHandComContactWrench.getVal(4));
        rightHandContactWrench.setVal(5, rightHandComContactWrench.getVal(5));
        
        baseForceAndJointTorques = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model());
        KinDynModel.kinDynComp.inverseDynamics(baseAcc, d2q, linkNetWrench, baseForceAndJointTorques);
        inverseDynamicsJointTorques(:, t) = baseForceAndJointTorques.jointTorques.toMatlab;
        baseWrench(:, t) = baseForceAndJointTorques.baseWrench.toMatlab()';
        
        inertialTrqs(:, t) = inverseDynamicsJointTorques(:, t) - gravityTrqs(:, t) - biasTrqs(:, t);
        
    end
    
end