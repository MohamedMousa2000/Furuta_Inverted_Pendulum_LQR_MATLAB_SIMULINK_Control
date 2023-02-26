
%% State Space (System) parameters
%Arm Length
L_r = 0.1;
%Pendulum Length
L_p = 0.145; 
%Arm mass
m_r = 0.024;
%Pendulum mass                                               %m_p = 0.0223; Before cutting the pendulum
m_p = 0.032; % After cutting the pendulum
%Coupler mass
m_c = 0.08;
%Coupler redius
r_c =0.009;
%Length from the coupling point to the C.G. of the pendulum
l = L_p/2;
%Motor Inertia
J_r = 0.5*(0.072/2+0.005)*(0.006^2);
%Arm Inertia
J_arm=(((m_r)*L_r^2)/3)+J_r;
%Coupler Inertia
J_c = 0.5*(m_c*(r_c^2));
%Independent Pendulum Inertia
J_1 = (1/12)*m_p*(L_p^2);
%Total Pendulum Inertia
J_pen = J_c+J_1;
%Gravity
g=9.81;
%For A_matrix
A_23 = (-g*(m_p^2)*(l^2)*(L_r))/(J_arm*(J_pen+(m_p*(l^2)))+J_pen*m_p*(L_r^2));
A_43 =((J_arm + m_p*(L_r^2))*m_p*l*g)/(J_arm*(J_pen+(m_p*(l^2)))+J_pen*m_p*(L_r^2));
%For B_matrix
B2 = (J_pen + m_p*(l^2))/(J_arm*(J_pen + m_p*(l^2))+(J_pen*m_p*(L_r^2)));
B4 = (-m_p*l*L_r)/(J_arm*(J_pen + m_p*(l^2))+(J_pen*m_p*(L_r^2)));

%% Non linear Parameters

M1 = J_arm + m_p*(l^2);
M2 = m_p*l_p*l;
M3 = m_p*l_p*l;
M4 = J_pen+m_p*(l^2);
C1 = 0.5*m_p*l^2;
C2_1 = -m_p*l*L_r;
C2_2 = 0.5*m_p*(l^2);
C3 = -0.5*m_p*(l^2);
G = -m_p*l*g;

Kt = 0.086;
Kb = 0.086;
Ra = 2.5;


%% State Space Matrices
A = [0 1 0    0;
     0 0 A_23 0;
     0 0 0    1;
     0 0 A_43 0
    ];

B = [0;
     B2;
     0;
     B4;
     ];

C=[ 1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

D=[ 0;
    0;
    0;
    0];

%% LQR Control Matricies

 Q = [1  0  0  0;
      0  0.1  0  0;
      0  0  1  0;
      0  0  0  0.1];
 R = 180000000;
 %% Getting Gains and Apply an impulse input
K = lqr(A,B,Q,R)
A_cl = A-B*K;
sys1 = ss(A-B*K,B,C,D);
impulse(sys1);
