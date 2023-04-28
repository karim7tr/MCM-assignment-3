function J = GetJacobian(biTei, bTe, jointType)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix

numberOfLinks=length(jointType);

J=zeros(6,numberOfLinks); %initialization

    for i=1:numberOfLinks
        %We first find the angular part of the Jacobian A.

        if(jointType(i)==0)   %In case of Revolut joitn RJ.
            J(1:3,i)=bTe(1:3,3,i);  % The angular part in case of Revolut joint is simply the axis of rotation.
        else   %In case of PJ.
            J(1:3,i)=[0;0;0]; %it is zero in case of Presmatic joint.
        end

        %Then we find the Linear part L.
        if(jointType(i)==0)  %RJ
            J(4:6,i)=cross(bTe(1:3,3,i),(bTe(1:3,4,7)-bTe(1:3,4,i)));   %it is the cross product of rotation axis wrt base frame and the distance vector.
        else %PJ
            J(4:6,i)=bTe(1:3,3,i);  % The Linear part in case of Presmatic joint is simply the axis of translation.
        end
    end
end