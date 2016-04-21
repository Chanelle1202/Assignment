clear
clc

%% Training data for ANFIS inverse kinematics system

l1=65; l2=155; l3=160; l5=100;


theta1 = linspace(-pi/2, pi/2, 50); % all possible theta1 values
theta2 = linspace(0, pi, 50); % all possible theta2 values
theta3 = linspace(-pi/2, 0, 50); %all possible theta3 values
theta4 = linspace(-pi/2, 0, 50); %all possible theta4 values


[THETA1, THETA2, THETA3, THETA4] = ndgrid(theta1, theta2, theta3, theta4); % generate a grid of theta values


%Forward Kinematics Equations
X = cos(THETA1).*(l3*cos(THETA2+THETA3)+l2*cos(THETA2)+l5*cos(THETA2+THETA3+THETA4));
Y = sin(THETA1).*(l3*cos(THETA2+THETA3)+l2*cos(THETA2)+l5*cos(THETA2+THETA3+THETA4));
Z = l1 + l3*sin(THETA2+THETA3)+l2*sin(THETA2)+l5*sin(THETA2+THETA3+THETA4);
B = -(THETA2+THETA3+THETA4);

data1 = [X(:) Y(:) THETA1(:)]; % create x-y-theta1 dataset
data2 = [X(:) Z(:) B(:) THETA2(:)]; % create x-y-theta2 dataset
data3 = [X(:) Z(:) B(:) THETA3(:)]; % create x-y-theta3 dataset
data4 = [X(:) Z(:) B(:) THETA4(:)]; % create x-y-theta4 dataset

training_data1 = data1(1:2:end,:);
training_data2 = data2(1:2:end,:);
training_data3 = data3(1:2:end,:);
training_data4 = data4(1:2:end,:);

validation_data1 = data1(2:2:end, :); % create x-y-theta1 dataset
validation_data2 = data2(2:2:end, :); % create x-y-theta2 dataset
validation_data3 = data3(2:2:end, :); % create x-y-theta3 dataset
validation_data4 = data4(2:2:end, :); % create x-y-theta4 dataset


%% Training

theta1_radii = 0.5;
theta2_radii = 0.5;
theta3_radii = 0.5;
theta4_radii = 0.5;

tic;

%Genfis2
input_fismat1 = genfis2(training_data1(:,1:2), training_data1(:,3), theta1_radii);
input_fismat2 = genfis2(training_data2(:,1:3), training_data2(:,4), theta2_radii);
input_fismat3 = genfis2(training_data3(:,1:3), training_data3(:,4), theta3_radii);
input_fismat4 = genfis2(training_data4(:,1:3), training_data4(:,4), theta4_radii);





%ANFIS
numEpochs = 500;

fprintf('-->%s\n','Start training first ANFIS network.')
[training_fismat1,trnErr1,ss1,validation_fismat1,valErr1]=anfis(training_data1,input_fismat1,numEpochs,[0,0,0,0],validation_data1); % train first ANFIS network

fprintf('-->%s\n','Start training second ANFIS network.')
[training_fismat2,trnErr2,ss2,validation_fismat2,valErr2]=anfis(training_data2,input_fismat2,numEpochs,[0,0,0,0],validation_data2); % train second ANFIS network

fprintf('-->%s\n','Start training third ANFIS network.')
[training_fismat3,trnErr3,ss3,validation_fismat3,valErr3]=anfis(training_data3,input_fismat3,numEpochs,[0,0,0,0],validation_data3); % train third ANFIS network

fprintf('-->%s\n','Start training fourth ANFIS network.')
[training_fismat4,trnErr4,ss4,validation_fismat4,valErr4]=anfis(training_data4,input_fismat4,numEpochs,[0,0,0,0],validation_data4); % train fourth ANFIS network

t=toc;

% Plot membership functions before training
figure(1)

subplot(2, 2, 1)
[ x1,mf1 ] = plotmf(fismat1,'input',1);
plot(x1,mf1);
xlabel('');
ylabel('Degree of Membership');

subplot(2, 2, 2)
[ x2,mf2 ] = plotmf(fismat2,'input',1);
plot(x2,mf2);
xlabel('');
ylabel('Degree of Membership');

subplot(2, 2, 3)
[ x3,mf3 ] = plotmf(fismat3,'input',1);
plot(x3,mf3);
xlabel('');
ylabel('Degree of Membership');

subplot(2, 2, 4)
[ x4,mf4 ] = plotmf(fismat4,'input',1);
plot(x4,mf4);
xlabel('');
ylabel('Degree of Membership');

% Plot membership functions after training
figure(2)

subplot(2, 2, 1)
[ y1,nf1 ] = plotmf(validation_fismat1,'input',1);
plot(y1,nf1);
xlabel('');
ylabel('Degree of Membership');

subplot(2, 2, 2)
[ y2,nf2 ] = plotmf(validation_fismat2,'input',1);
plot(y2,nf2);
xlabel('');
ylabel('Degree of Membership');

subplot(2, 2, 3)
[ y3,nf3 ] = plotmf(validation_fismat3,'input',1);
plot(y3,nf3);
xlabel('');
ylabel('Degree of Membership');

subplot(2, 2, 4)
[ y3,nf3 ] = plotmf(validation_fismat3,'input',1);
plot(y3,nf3);
xlabel('');
ylabel('Degree of Membership');
%% Checking Errors for Over/Underfitting

% %% Checking Errors for Over/Underfitting
epochs = 1:numEpochs;

figure(3) % new figure
%theta1
subplot(2,2,1);
plot(epochs, trnErr1, epochs, valErr2)

title('Theta1')
ylabel('Error')
xlabel('Epochs')
legend('training','validation')

%theta2
subplot(2,2,2);
plot(epochs, trnErr2, epochs, valErr2)

title('Theta2')
ylabel('Error')
xlabel('Epochs')
legend('training','validation')

%theta3
subplot(2,2,2);
plot(epochs, trnErr3, epochs, valErr3)

title('Theta3')
ylabel('Error')
xlabel('Epochs')
legend('training','validation')

%theta4
subplot(2,2,4);
plot(epochs, trnErr4, epochs, valErr4)

title('Theta4')
ylabel('Error')
xlabel('Epochs')
legend('training','validation')


%% Calculating the RMSE in Joint Space

%Theta1
trnOut1=evalfis(training_data1(:,1:4),training_fismat1);
trnRMSE1=norm(trnOut1-training_data1(:,5))/sqrt(length(trnOut1));
chkOut1=evalfis(validation_data1(:,1:4),validation_fismat1);
chkRMSE1=norm(chkOut1-validation_data1(:,5))/sqrt(length(chkOut1));

%Theta2
trnOut2=evalfis(training_data2(:,1:4),training_fismat2);
trnRMSE2=norm(trnOut2-training_data2(:,5))/sqrt(length(trnOut2));
chkOut2=evalfis(validation_data2(:,1:4),validation_fismat2);
chkRMSE2=norm(chkOut2-validation_data2(:,5))/sqrt(length(chkOut2));

%Theta1
trnOut3=evalfis(training_data3(:,1:4),training_fismat3);
trnRMSE3=norm(trnOut3-training_data3(:,5))/sqrt(length(trnOut3));
chkOut3=evalfis(validation_data3(:,1:4),validation_fismat3);
chkRMSE3=norm(chkOut3-validation_data3(:,5))/sqrt(length(chkOut3));

%Theta1
trnOut4=evalfis(training_data4(:,1:4),training_fismat4);
trnRMSE4=norm(trnOut4-training_data4(:,5))/sqrt(length(trnOut4));
chkOut4=evalfis(validation_data4(:,1:4),validation_fismat4);
chkRMSE4=norm(chkOut4-validation_data4(:,5))/sqrt(length(chkOut4));


%% Errors in Cartesian Space for Theta2

X_out = cos(chkOut1).*(l3*cos(chkOut2+chkOut3)+l2*cos(chkOut2)+l5*cos(chkOut2+chkOut3+chkOut4));
Y_out = sin(chkOut1).*(l3*cos(chkOut2+chkOut3)+l2*cos(chkOut2)+l5*cos(chkOut2+chkOut3+chkOut4));
Z_out = l1 + l3*sin(chkOut2+chkOut3)+l2*sin(chkOut2)+l5*sin(chkOut2+chkOut3+chkOut4);

X_error = X_out - validation_data2(:,1);
Y_error = Y_out - validation_data2(:,2);
Z_error = Z_out - validation_data2(:,3);

cartesian_error = ((X_error).^2 + (Y_error).^2 + (Z_error).^2).^0.5;

cartesian_errorRMSE = norm(cartesian_error)/sqrt(length(cartesian_error));

figure(4)
subplot(2, 2, 1)
scatter(training_data1(:,4), cartesian_error, 5);
xlabel('Theta1 values (radians)');

subplot(2, 2, 2)
scatter(training_data2(:,4), cartesian_error, 5);
xlabel('Theta2 values (radians)');

subplot(2, 2, 3)
scatter(training_data3(:,4), cartesian_error, 5);
xlabel('Theta3 values (radians)');

subplot(2, 2, 4)
scatter(training_data4(:,4), cartesian_error, 5);
xlabel('Theta4 values (radians)');
ylabel('Cartesian Error (mm)');


