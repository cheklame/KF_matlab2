clear all;
figure(1);clf

%%droite (cas linéaire vitesse constante et acceleration null)
%% [4,4]

fe = 30; % fréquence d'échantillonnage
temps = 10; % temps total de mesure en seconde


vx = 3;
vy = 2;
Tx = 4;
Ty = 3;

vec_etat = zeros(temps*fe, 4); % [x ; y; vx, vy]
vec_etat(1:1:temps*fe,3) = vx;
vec_etat(1:1:temps*fe,4) = vy;
i = 2;
for t = 0:1/fe:temps-2/fe
   vec_etat(i,1) = vec_etat(i-1,1) + Tx * vec_etat(i-1,3);
   vec_etat(i,2) = vec_etat(i-1,2) + Ty * vec_etat(i-1,4);
   i = i+1;
end

%%



%%


%%Kalman Filter

%%declaration and initialisation of variables

A = [1 0 Tx 0; 0 1 0 Ty; 0 0 1 0; 0 0 0 1];
C = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
sig1 = 2;
noise = 2;
R = zeros(4,4);
R(1,1) = sig1;
R(2,2) = sig1;
R(3,3) = sig1;
R(4,4) = sig1;
Q = zeros(4,4);
K = zeros(4);
K_save(1,:) = K(1);
X = [0 ; 0 ; 0; 0];
Xp = [0 ; 0 ; vx; vy];
P = eye(4);
X_sav(1,:) = X;
k = 2;
l = 2;
%%%
%%%---mesurement with noise-----------
j = 1;
for t1 = 0:1/fe:temps-1/fe
 Y(j,:) = C * vec_etat(j,:)' + noise;
 j = j+1;
end


%%begining of KF
for t1 = 0:1/fe:temps-2/fe

%%prediction next state and next covariance
   X = A*X;
   P = A*P*A' + Q;

%update the state and covarience
    K = P * C' * inv(C * P * C' + R);
    X = X + K * (Y(l,:)' - C * X);
    P = P - K * C * P;
        
    K_save(k,1) = K(1);
    X_sav(k,:) =  X;
    %p_sav(k,:) =  X;
    
    
l = l+1;
k = k+1;
end

taille = size(vec_etat);
s = taille(1);
t2=(0:1/fe:(s-1)/fe);
subplot(1, 3, 1);hold on;
plot(vec_etat(:,1),vec_etat(:,2),'r');hold on;
plot(X_sav(:,1),X_sav(:,2),'g');
legend('graphe en x/y');
xlabel('X');ylabel('Y');

subplot(1, 3, 2);hold on;
plot(t2,vec_etat(:,1),'r');hold on;
plot(t2,X_sav(:,1),'g');
legend('graphe en t/X');
xlabel('temps');ylabel('X');

subplot(1, 3, 3);hold on;
plot(t2,vec_etat(:,2),'r');hold on;
plot(t2,X_sav(:,2),'g');
legend('graphe en t/y');
xlabel('temps');ylabel('Y');




%%
%%% stright curve ax != 0 and ay = 0 [5,5]
clear all;
figure(2);clf

fe = 30; % fréquence d'échantillonnage
temps = 10; % temps total de mesure en seconde


vx = 3;
vy = 2;
Tx = 4;
Ty = 3;
Tvx = 2;
ax = 1;
vec_etat = zeros(temps*fe, 5); % [x ; y; vx, vy]
vec_etat(1:1:temps*fe,4) = vy;
vec_etat(1:1:temps*fe,5) = ax;
i = 2;
for t = 0:1/fe:temps-2/fe
   vec_etat(i,1) = vec_etat(i-1,1) + Tx * vec_etat(i-1,3);
   vec_etat(i,2) = vec_etat(i-1,2) + Ty * vec_etat(i-1,4);
   vec_etat(i,3) = vec_etat(i-1,3) + Tvx * vec_etat(i-1,5);
   
   i = i+1;
end

%%Kalman Filter

%%declaration and initialisation of variables

A = [1 0 0 0 0; 0 1 0 0 0 ; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 1];
C = [1 0 Tx 0 0; 0 1 0 Ty 0; 0 0 1 0 Tvx; 0 0 0 1 0; 0 0 0 0 1];
F = eye(5);
sig1 = 2;
noise = 2;
R = zeros(5,5);
R(1,1) = sig1;
R(2,2) = sig1;
R(3,3) = sig1;
R(4,4) = sig1;
R(5,5) = sig1;
Q = zeros(5,5);
K = zeros(5);
K_save(1,:) = K(1);
X = [0 ; 0 ; 0; 0; 0];
Xp = [0 ; 0 ; 0; 0; 0];
P = eye(5);
X_sav(1,:) = X;
k = 2;
l = 2;
%%%
%%%---mesurement with noise-----------
j = 1;
for t1 = 0:1/fe:temps-1/fe
 Y(j,:) = C * vec_etat(j,:)' + noise;
 j = j+1;
end


%%begining of KF
for t1 = 0:1/fe:temps-2/fe

%%prediction next state and next covariance
   X = A*X;
   P = A*P*A' + Q;

%update the state and covarience
    K = P * C' * inv(C * P * C' + R);
    X = X + K * (Y(l,:)' - C * X);
    P = P - K * C * P;
        
    K_save(k,1) = K(1);
    X_sav(k,:) =  X;
    %p_sav(k,:) =  X;
    
    
l = l+1;
k = k+1;
end

taille = size(vec_etat);
s = taille(1);
t2=(0:1/fe:(s-1)/fe);
subplot(1, 3, 1);hold on;
plot(vec_etat(:,1),vec_etat(:,2),'r');hold on;
plot(X_sav(:,1),X_sav(:,2),'g');
legend('graphe en x/y');
xlabel('X');ylabel('Y');

subplot(1, 3, 2);hold on;
plot(t2,vec_etat(:,1),'r');hold on;
plot(t2,X_sav(:,1),'g');
legend('graphe en t/X');
xlabel('temps');ylabel('X');


subplot(1, 3, 3);hold on;
plot(t2,vec_etat(:,2),'r');hold on;
plot(t2,X_sav(:,2),'g');
legend('graphe en t/y');
xlabel('temps');ylabel('Y');
