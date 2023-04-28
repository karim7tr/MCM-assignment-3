%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix
clc;
clear;
close all;
addpath('include');
% The same model of assignment 2
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = zeros(numberOfLinks,1); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base
%disp(numberOfLinks);
% joint configurations : to chose a configurations just uncomment. 
q1 = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];
q2 = [1.3, 0.4, 0.1, 0, 0.5, 1.1, 0];
q3 = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
q4 = [2, 2, 2, 2, 2, 2, 2];

% Compute direct geometry for each configuration: for specific
% configuration,uncomment.
biTei1 = GetDirectGeometry(q1,geom_model,linkType);
biTei2 = GetDirectGeometry(q2,geom_model,linkType);
biTei3 = GetDirectGeometry(q3,geom_model,linkType);
biTei4 = GetDirectGeometry(q4,geom_model,linkType);

% Compute the transformation w.r.t. the base

for i =1:numberOfLinks
    bTi1(:,:,i)= GetTransformationWrtBase(biTei1,i);
    bTi2(:,:,i)= GetTransformationWrtBase(biTei2,i);
    bTi3(:,:,i)= GetTransformationWrtBase(biTei3,i);
    bTi4(:,:,i)= GetTransformationWrtBase(biTei4,i);
end

% computing end effector jacobian :  to chose a configurations uncomment.
    j1=GetJacobian(biTei1, bTi1,linkType);
    j2=GetJacobian(biTei2, bTi2,linkType);
    j3=GetJacobian(biTei3, bTi3,linkType);
    j4=GetJacobian(biTei4, bTi4,linkType);
