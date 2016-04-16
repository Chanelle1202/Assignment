%% Genfis2 2Link GA

lb = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1]; %lower constraint as radii values must be positive
ub = [0.9; 0.9; 0.9; 0.9; 0.9; 0.9]; %upper constraint as radii values must be <= 1
opts = optimoptions(@ga, 'MaxTime', 60, 'PlotFcn',{@gaplotbestf,@gaplotmaxconstr},'Display','iter');
[x, fval, exitflag, output, population] = ga(@ANFIS_IK_2Link_Genfis2, 6, [],[],[],[],lb,ub, [], opts);