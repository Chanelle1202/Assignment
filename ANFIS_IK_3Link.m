clear
clc

%% Training data for ANFIS inverse kinematics system

l1=65; l2=155; l3=160; l5=100;

theta2 = linspace(0, 3*pi/4, 50); % all possible theta2 values
theta3 = linspace(-pi/2, 0, 50); %Restricted theta3 values for elbow up configurations
theta4 = linspace(-pi/2, 0, 50); %Restricted theta4 values for wrist up configurations

[THETA2, THETA3, THETA4] = ndgrid(theta2, theta3, theta4); % generate a grid of theta values

%Forward Kinematics Equations
X = l3*cos(THETA2+THETA3)+l2*cos(THETA2)+l5*cos(THETA2+THETA3+THETA4);
Z = l1 + l3*sin(THETA2+THETA3)+l2*sin(THETA2)+l5*sin(THETA2+THETA3+THETA4);
B = -(THETA2+THETA3+THETA4); %Pitch constraint

data2 = [X(:) Z(:) B(:) THETA2(:)]; 
data3 = [X(:) Z(:) B(:) THETA3(:)]; 
data4 = [X(:) Z(:) B(:) THETA4(:)]; 

training_data2 = data2(1:2:end,:);
training_data3 = data3(1:2:end,:);
training_data4 = data4(1:2:end,:);

validation_data2 = data2(2:2:end, :); 
validation_data3 = data3(2:2:end, :); 
validation_data4 = data4(2:2:end, :); 

%% Training

%Genfis1
numMFs = 5;

input_fismat2 = genfis1(training_data2, numMFs);
input_fismat3 = genfis1(training_data3, numMFs);
input_fismat4 = genfis1(training_data4, numMFs);

%ANFIS
numEpochs = 200;

fprintf('-->%s\n','Start training theta2 ANFIS network.')
[training_fismat2,trnErr2,ss2,validation_fismat2,valErr2]=anfis(training_data2,input_fismat2,numEpochs,[0,0,0,0],validation_data2); % train second ANFIS network

fprintf('-->%s\n','Start training theta3 ANFIS network.')
[training_fismat3,trnErr3,ss3,validation_fismat3,valErr3]=anfis(training_data3,input_fismat3,numEpochs,[0,0,0,0],validation_data3); % train third ANFIS network

fprintf('-->%s\n','Start training theta4 ANFIS network.')
[training_fismat4,trnErr4,ss4,validation_fismat4,valErr4]=anfis(training_data4,input_fismat4,numEpochs,[0,0,0,0],validation_data4); % train fourth ANFIS network

fprintf('-->%s\n','Finished training networks.')
%% Checking Errors for Over/Underfitting

% epochs = 1:numEpochs;
% 
% figure(1) % new figure
% title('Training vs Validation Error - 3LinkGenfis150Thetas100Epochs3MFs');
% 
% theta2
% subplot(1,3,1);
% plot(epochs, trnErr2, epochs, valErr2)
% 
% title('Theta2')
% ylabel('Error')
% xlabel('Epochs')
% legend('training','validation')
% 
% theta3
% subplot(1,3,2);
% plot(epochs, trnErr3, epochs, valErr3)
% 
% title('Theta3')
% ylabel('Error')
% xlabel('Epochs')
% legend('training','validation')
% 
% theta4
% subplot(1,3,3);
% plot(epochs, trnErr4, epochs, valErr4)
% 
% title('Theta4')
% ylabel('Error')
% xlabel('Epochs')
% legend('training','validation')

%% Using evalfis to evaluate the trained system

%Theta2
trnOut2=evalfis(training_data2(:,1:3),training_fismat2);
trnRMSE2=norm(trnOut2-training_data2(:,4))/sqrt(length(trnOut2));
chkOut2=evalfis(validation_data2(:,1:3),validation_fismat2);
chkRMSE2=norm(chkOut2-validation_data2(:,4))/sqrt(length(chkOut2));

%Theta3
trnOut3=evalfis(training_data3(:,1:3),training_fismat3);
trnRMSE3=norm(trnOut3-training_data3(:,4))/sqrt(length(trnOut3));
chkOut3=evalfis(validation_data3(:,1:3),validation_fismat3);
chkRMSE3=norm(chkOut3-validation_data3(:,4))/sqrt(length(chkOut3));

%Theta4
trnOut4=evalfis(training_data4(:,1:3),training_fismat4);
trnRMSE4=norm(trnOut4-training_data4(:,4))/sqrt(length(trnOut4));
chkOut4=evalfis(validation_data4(:,1:3),validation_fismat4);
chkRMSE4=norm(chkOut4-validation_data4(:,4))/sqrt(length(chkOut4));

%% Checking Cartesian Error

X_out = l3*cos(chkOut2+chkOut3)+l2*cos(chkOut2)+l5*cos(chkOut2+chkOut3+chkOut4);
Z_out = l1 + l3*sin(chkOut2+chkOut3)+l2*sin(chkOut2)+l5*sin(chkOut2+chkOut3+chkOut4);

X_error = X_out - validation_data2(:,1);
Z_error = Z_out - validation_data2(:,2);

cartesian_error = ((X_error).^2 + (Z_error).^2).^0.5;

cartesian_errorRMSE = norm(cartesian_error)/sqrt(length(cartesian_error));

% scatter(training_data4(:,4), cartesian_error);
