clear
clc


%% Training

partitionNumLoop = [10,20,50,100,200,500];

for i=partitionNumLoop %cycle through partition numbers
    for j=2:10 %cycle through number of membership functions
        numEpochs = 500;
        partitionNum = i;
        numMFs = j;
        
        tic;
        [trnRMSE2, chkRMSE2, trnRMSE3, chkRMSE3, cartesian_error] = ANFIS_IK_2Link_Genfis1(partitionNum, numMFs, numEpochs, 0);      
        
        t = toc;
        cartesian_error_min = min(cartesian_error);
        cartesian_error_max = max(cartesian_error);
     
        cartesian_errorRMSE = norm(cartesian_error)/sqrt(length(cartesian_error));
        
        %% Printing to file
        
        A = [partitionNum, numEpochs, numMFs, trnRMSE2, chkRMSE2, trnRMSE3, chkRMSE3, cartesian_errorRMSE, cartesian_error_min, cartesian_error_max, t];
        
        fileID = fopen('Genfis1errors.txt','at');
        fprintf(fileID,'%d |\t%d |\t%d |\t%.4f |\t%.4f |\t%.4f |\t%.4f |\t%.4f |\t%.4f |\t%.4f |\t\t%.2f \n',A);
        fclose(fileID);
    end
end




