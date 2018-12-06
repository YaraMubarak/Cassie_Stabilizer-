function tau = studentController(t, s, model, params)
% Modify this code to calculate the joint torques
% t - time
% s - state of the robot
% model - struct containing robot properties
% params - user defined parameters in studentParams.m
% tau - 10x1 vector of joint torques

% State vector components ID

q = s(1 : model.n);
dq = s(model.n+1 : 2*model.n);

%% [Control #1] zero control
% tau = zeros(10,1);

%% [Control #2] High Gain Joint PD control on all actuated joints
kp = 400 ;
kd = 130 ;
x0 = getInitialState(model);
q0 = x0(1:model.n) ;


%Michael's Additions%%%%%%%%%%%%%
tau = [];

f_ext = ExternalForce(t, q,model);
[H,C] = HandC(model,q, dq, f_ext);
[P_COM,V_COM] = computeComPosVel(q,dq,model);
J_COM = computeComJacobian(q,model);
[P_Foot1F,P_Foot1B,P_Foot2F,P_Foot2B] = computeFootPositions(q,model);
[J_Foot1F,J_Foot1B,J_Foot2F,J_Foot2B]  = computeFootJacobians(q,dq,model);
Gravity = get_gravity(model);

%End of Michael's Additions%%%%%%
%Gravity offset, attempt to actuate knees to offset gravity since it's
%always passively there
%So far, only in the Z axis (can expand to other axis later)
knees = [4 9]; toes = [5 10]; flexes = [3 8];
kG = 1;
tau = zeros(10,1);
tau(knees) = C(3)*kG;
tau(toes) = -C(3)*kG;
tau(flexes) = -C(3)*kG;

%General Control (vanilla)
tau = tau+(-kp*(q(model.actuated_idx)-q0(model.actuated_idx)) - kd*dq(model.actuated_idx)) ;


