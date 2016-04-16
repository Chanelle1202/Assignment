%% Number of Epochs Eperiement

numEpochsLoop = [50,100,500, 1000];

for i=numEpochsLoop %cycle through partition numbers
        numEpochs = i;
        partitionNum = 100;
        numMFs = 2;
             
        tic;
        [trnRMSE2, chkRMSE2, trnRMSE3, chkRMSE3, cartesian_error] = ANFIS_IK_2Link_Genfis1(partitionNum, numMFs, i, 0);      
        
        t = toc;
        cartesian_error_min = min(cartesian_error);
        cartesian_error_max = max(cartesian_error);
     
        cartesian_errorRMSE = norm(cartesian_error)/sqrt(length(cartesian_error));
        
        %% Printing to file
        
        A = [partitionNum, numEpochs, numMFs, trnRMSE2, chkRMSE2, trnRMSE3, chkRMSE3, cartesian_errorRMSE, cartesian_error_min, cartesian_error_max, t];
        
        fileID = fopen('Genfis1errors-Epochs.txt','at');
        fprintf(fileID,'%d |\t%d |\t%d |\t%.4f |\t%.4f |\t%.4f |\t%.4f |\t%.4f |\t%.4f |\t%.4f |\t\t%.2f \n',A);
        fclose(fileID);

end