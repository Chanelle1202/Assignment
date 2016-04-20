%% MFs in Genfis1
%cartRMSE time

ten = [35.0791 ,		0.32;
88.9466 ,		0.44 ;
168.6123, 	0.99 ;
194.8358, 	2.03 ;
106.5546, 	3.91 ;
191.3857, 	7.09 ;
164.0823, 	11.81;
161.5942, 	18.56 ;
199.808 , 28.28  ];

twenty = [12.4193 ,  0.74; 
5.5069   ,  1.74; 
11.8301  ,  3.95; 
65.1519  ,  8.43; 
35.6708  ,  15.52; 
61.4179  ,  28.16; 
41.7306  ,  48.13; 
102.77   ,  73.61; 
0.0007   ,  111.39 ];

fifty = [7.7817 , 	  4.46 ;
5.1415  ,	 10.80 ;
2.2461  ,	 23.72 ;
1.5393  ,    50.42 ;
7.5921  ,	 96.92 ;
2.1075  ,	175.22 ;
2.1102  ,	292.86 ;
2.8006  ,	468.32 ;
2.8987  ,	699.17 ];

hundred = [5.5590  ,      18.10 ;
3.6464  ,	  42.97 ;
1.9688  ,	  95.06 ;
1.4229  ,	 201.28 ;
0.9550  ,     386.25 ;
1.5349  ,	 700.12 ;
1.1876  ,	1167.94 ;
0.7292  ,    1854.52 ;
1.7005  ,	2776.96 ];

two_hundred = [5.6355 ,   76.53 ;
3.2618  ,  175.64 ;
1.8952  ,  387.51 ;
1.6488  ,  816.60 ;
0.8359  , 1554.44 ];

%% Number of MFS against time and catesian RMSE
numMFs = 2:10;

figure(1)
subplot(1,2,1)
plot(numMFs, ten(:,1)'); hold on;
plot(numMFs, twenty(:,1)'); hold on;
plot(numMFs, fifty(:,1)'); hold on
plot(numMFs,  hundred(:,1)'); hold on;
xlabel('Number of MFs');
ylabel('Cartesian RMSE');

subplot(1, 2,2)
plot(numMFs, ten(:,2)'); hold on;
plot(numMFs, twenty(:,2)'); hold on;
plot(numMFs, fifty(:,2)'); hold on
plot(numMFs,  hundred(:,2)'); hold on;
xlabel('Number of MFs');
ylabel('Time Taken');

legend('10 Data Points', '20 Data Points', '50 Data Points', '100 Data Points');


