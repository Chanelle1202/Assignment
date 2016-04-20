%% Genfis2 2Link GA

lb = [0.1; 0.1; 0.1]; %lower constraint as radii values must be positive
ub = [0.9; 0.9; 0.9]; %upper constraint as radii values must be <= 1
opts = gaoptimset('TimeLimit', 36000, 'PopulationSize', 15, 'PlotFcn',{@gaplotbestf});
[x, fval, exitflag, output, population] = ga(@ANFIS_IK_3Link_Genfis2, 3, [],[],[],[],lb,ub, [], opts);