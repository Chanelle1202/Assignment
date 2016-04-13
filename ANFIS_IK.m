clear
clc

%% Training data for ANFIS inverse kinematics system

l1=65; l2=155; l3=160; l5=100;


theta1 = linspace(-pi/2, pi/2, 20); % all possible theta1 values
theta2 = linspace(0, pi, 20); % all possible theta2 values
theta3 = linspace(-pi/2, pi/2, 20); %all possible theta3 values
theta4 = linspace(-pi/2, pi/2, 20); %all possible theta4 values


[THETA1, THETA2, THETA3, THETA4] = ndgrid(theta1, theta2, theta3, theta4); % generate a grid of theta values


%Forward Kinematics Equations
X = cos(THETA1).*(l3*cos(THETA2+THETA3)+l2*cos(THETA2)+l5*cos(THETA2+THETA3+THETA4));
Y = sin(THETA1).*(l3*cos(THETA2+THETA3)+l2*cos(THETA2)+l5*cos(THETA2+THETA3+THETA4));
Z = l1 + l3*sin(THETA2+THETA3)+l2*sin(THETA2)+l5*sin(THETA2+THETA3+THETA4);

data1 = [X(:) Y(:) Z(:) THETA1(:)]; % create x-y-theta1 dataset
data2 = [X(:) Y(:) Z(:) THETA2(:)]; % create x-y-theta2 dataset
data3 = [X(:) Y(:) Z(:) THETA3(:)]; % create x-y-theta3 dataset
data4 = [X(:) Y(:) Z(:) THETA4(:)]; % create x-y-theta4 dataset

training_data1 = data1(1:2:end,:);
training_data2 = data2(1:2:end,:);
training_data3 = data3(1:2:end,:);
training_data4 = data4(1:2:end,:);

validation_data1 = data1(2:2:end, :); % create x-y-theta1 dataset
validation_data2 = data2(2:2:end, :); % create x-y-theta2 dataset
validation_data3 = data3(2:2:end, :); % create x-y-theta3 dataset
validation_data4 = data4(2:2:end, :); % create x-y-theta4 dataset


%% Training

%Genfis1
numMFs = 3;

input_fismat1 = genfis1(training_data1, numMFs);
input_fismat2 = genfis1(training_data2, numMFs);
input_fismat3 = genfis1(training_data3, numMFs);
input_fismat4 = genfis1(training_data4, numMFs);

%Genfis2
% input_fismat1 = genfis2(training_data1(:,1:3), training_data1(:,4), 0.5);
% input_fismat2 = genfis2(training_data2(:,1:3), training_data2(:,4), 0.5);
% input_fismat3 = genfis2(training_data3(:,1:3), training_data3(:,4), 0.5);
% input_fismat4 = genfis2(training_data4(:,1:3), training_data4(:,4), 0.5);

%ANFIS
numEpochs = 100;

fprintf('-->%s\n','Start training first ANFIS network.')
[training_fismat1,trnErr1,ss1,validation_fismat1,valErr1]=anfis(training_data1,input_fismat1,numEpochs,[1,1,1,1],validation_data1); % train first ANFIS network

fprintf('-->%s\n','Start training second ANFIS network.')
[training_fismat2,trnErr2,ss2,validation_fismat2,valErr2]=anfis(training_data2,input_fismat2,numEpochs,[1,1,1,1],validation_data2); % train second ANFIS network

fprintf('-->%s\n','Start training third ANFIS network.')
[training_fismat3,trnErr3,ss3,validation_fismat3,valErr3]=anfis(training_data3,input_fismat3,numEpochs,[1,1,1,1],validation_data3); % train third ANFIS network

fprintf('-->%s\n','Start training fourth ANFIS network.')
[training_fismat4,trnErr4,ss4,validation_fismat4,valErr4]=anfis(training_data4,input_fismat4,numEpochs,[1,1,1,1],validation_data4); % train fourth ANFIS network


%% Checking Errors for Over/Underfitting

epochs = 1:numEpochs;

figure % new figure
plot(epochs, trnErr1, epochs, valErr1)

title('T1Genfis120T100E3MF - Training and Validation Error')
ylabel('Error')
xlabel('Epochs')
legend('training','validation')

figure % new figure
plot(epochs, trnErr2, epochs, valErr2)

title('T2Genfis120T100E3MF - Training and Validation Error')
ylabel('Error')
xlabel('Epochs')
legend('training','validation')


figure % new figure
plot(epochs, trnErr3, epochs, valErr3)

title('T3Genfis120T100E3MF - Training and Validation Error')
ylabel('Error')
xlabel('Epochs')
legend('training','validation')

figure % new figure
plot(epochs, trnErr4, epochs, valErr4)

title('T4Genfis120T100E3MF - Training and Validation Error')
ylabel('Error')
xlabel('Epochs')
legend('training','validation')

%% Plotting output against training data
figure

anfis_test1=evalfis([data1(:,1), data1(:,2), data1(:,3)] ,training_fismat1);

anfis_test1_y = anfis_test1*(180/pi);

training_data1_y = training_data1(1:10,4)*(180/pi);

validation_data1_y = validation_data1(1:10,4)*(180/pi);


%Compare training and checking data to the fuzzy approximation:
plot(training_data1_x, training_data1_y, 'o', validation_data1_x, validation_data1_y, 'x', anfis_test1_x, anfis_test1_y,'-')

hold on;

%% Using evalfis to evaluate the trained system

%Theta1
trnOut1=evalfis(training_data1(:,1:3),training_fismat1);
trnRMSE1=norm(trnOut1-training_data1(:,4))/sqrt(length(trnOut1));
chkOut1=evalfis(validation_data1(:,1:3),validation_fismat1);
chkRMSE1=norm(chkOut1-validation_data1(:,4))/sqrt(length(chkOut1));

%Theta2
trnOut2=evalfis(training_data2(:,1:3),training_fismat2);
trnRMSE2=norm(trnOut2-training_data2(:,4))/sqrt(length(trnOut2));
chkOut2=evalfis(validation_data2(:,1:3),validation_fismat2);
chkRMSE2=norm(chkOut2-validation_data2(:,4))/sqrt(length(chkOut2));

%Theta1
trnOut3=evalfis(training_data3(:,1:3),training_fismat3);
trnRMSE3=norm(trnOut3-training_data3(:,4))/sqrt(length(trnOut3));
chkOut3=evalfis(validation_data3(:,1:3),validation_fismat3);
chkRMSE3=norm(chkOut3-validation_data3(:,4))/sqrt(length(chkOut3));

%Theta1
trnOut4=evalfis(training_data4(:,1:3),training_fismat4);
trnRMSE4=norm(trnOut4-training_data4(:,4))/sqrt(length(trnOut4));
chkOut4=evalfis(validation_data4(:,1:3),validation_fismat4);
chkRMSE4=norm(chkOut4-validation_data4(:,4))/sqrt(length(chkOut4));





