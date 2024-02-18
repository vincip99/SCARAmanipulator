%% Parameter Initialization
% Init the SCARA manipulator parameters as global variables
d0 = 1; a1 = .5; a2 = .5; l1 = .25; l2 = .25; 
ml1 = 20; ml2 = 20; ml3 = 10;
Il1 = 4; Il2 = 4; Il4 = 1;
kr1 = 1; kr2 = 1; kr3 = 50; kr4 = 20;
Im1 = .01; Im2 = .01; Im3 = .005; Im4 = .001;
Fm1 = .00005; Fm2 = .00005; Fm3 = .01; Fm4 = .005;

% numLinks
numLinks = 5;  

alpha = ;
beta = ;
gamma = ;

%% Computation of inertia matrix B
B = [alpha + beta + 2*gamma*]


