function [trnRMSE2, chkRMSE2, trnRMSE3, chkRMSE3, cartesian_error] = ANFIS_IK_2Link_Genfis1(partitionNum, numMFs, numEpochs, show)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here


fprintf('-->%d %d %d\n', partitionNum, numMFs, numEpochs);
%% Training data for ANFIS inverse kinematics system

l1=65; l2=155; l3=160;

theta2 = linspace(0, pi, partitionNum); % all possible theta2 values
theta3 = linspace(-pi/2,0, partitionNum); %all possible theta3 values

[THETA2, THETA3] = ndgrid(theta2, theta3); % generate a grid of theta values

%Forward Kinematics Equations
X = l3*cos(THETA2+THETA3)+l2*cos(THETA2);
Z = l1 + l3*sin(THETA2+THETA3)+l2*sin(THETA2);

data2 = [X(:) Z(:) THETA2(:)];
data3 = [X(:) Z(:) THETA3(:)];

training_data2 = data2(1:2:end,:);
training_data3 = data3(1:2:end,:);

validation_data2 = data2(2:2:end, :);
validation_data3 = data3(2:2:end, :);

%% Training

%Genfis1
input_fismat2 = genfis1(training_data2, numMFs);
input_fismat3 = genfis1(training_data3, numMFs);

%ANFIS

fprintf('-->%s\n','Start training theta2 ANFIS network.')
[training_fismat2,trnErr2,ss2,validation_fismat2,valErr2]=anfis(training_data2,input_fismat2,numEpochs,[0,0,0,0],validation_data2); % train second ANFIS network

fprintf('-->%s\n','Start training theta3 ANFIS network.')
[training_fismat3,trnErr3,ss3,validation_fismat3,valErr3]=anfis(training_data3,input_fismat3,numEpochs,[0,0,0,0],validation_data3); % train third ANFIS network

fprintf('-->%s\n','Finished training networks.')
% %% Checking Errors for Over/Underfitting
if show == 1
    epochs = 1:numEpochs;

    figure(1) % new figure

    %theta2
    subplot(1,2,1);
    plot(epochs, trnErr2, epochs, valErr2)

    title('Theta2')
    ylabel('Error')
    xlabel('Epochs')
    legend('training','validation')

    %theta3
    subplot(1,2,2);
    plot(epochs, trnErr3, epochs, valErr3)

    title('Theta3')
    ylabel('Error')
    xlabel('Epochs')
    legend('training','validation')
end


%% Root Mean Square Error in Joint Space

%Theta2
trnOut2=evalfis(training_data2(:,1:2),training_fismat2);
trnRMSE2=norm(trnOut2-training_data2(:,3))/sqrt(length(trnOut2));
chkOut2=evalfis(validation_data2(:,1:2),validation_fismat2);
chkRMSE2=norm(chkOut2-validation_data2(:,3))/sqrt(length(chkOut2));

%Theta3
trnOut3=evalfis(training_data3(:,1:2),training_fismat3);
trnRMSE3=norm(trnOut3-training_data3(:,3))/sqrt(length(trnOut3));
chkOut3=evalfis(validation_data3(:,1:2),validation_fismat3);
chkRMSE3=norm(chkOut3-validation_data3(:,3))/sqrt(length(chkOut3));

%% Errors in Cartesian Space for Theta2

X_out = l3*cos(chkOut2+chkOut3)+l2*cos(chkOut2);
Z_out = l1 + l3*sin(chkOut2+chkOut3)+l2*sin(chkOut2);

X_error = X_out - validation_data2(:,1);
Z_error = Z_out - validation_data2(:,2);

cartesian_error = ((X_error).^2 + (Z_error).^2).^0.5;

if show==1
    figure(2)
    stem3(training_data2(:,3), training_data3(:,3), cartesian_error);
end

end

