%%
clear all

%% Compile and run
% system('make testrig && ./bin/testrig');

%%
% load -ASCII Alin
% LA.X = Alin(:,1:23)';
% LA.U = Alin(:,24:27)';
% LA.XD = Alin(:,28:end)';
%%
load -ASCII Blin
LB.X = Blin(:,1:23)';
LB.U = Blin(:,24:27)';
LB.XD = Blin(:,28:end)';

%%
load -ASCII step_xdot
load -ASCII step_x
Qp = step_xdot(1:2:end,:)';
Qm = step_xdot(2:2:end,:)';
C = Qp/2e-3 - Qm/2e-3;
Cx = step_x';
Cxd = step_xdot';

%%
load -ASCII A
load -ASCII B
load -ASCII Q
load -ASCII R
load -ASCII r
load -ASCII L
load -ASCII Lr
load -ASCII eX
load -ASCII S


A = permute(reshape(A', 13, 13, []), [2 1 3]);
B = permute(reshape(B', 4, 13, []), [2 1 3]);
Q = permute(reshape(Q', 13, 13, []), [2 1 3]);
R = permute(reshape(R', 4, 4, []), [2 1 3]);
r = r';
eX = eX';
L = permute(reshape(L', 13, 4, []), [2 1 3]);
Lr = permute(reshape(Lr', 4, 4, []), [2 1 3]);
S = permute(reshape(S', 13, 13, []), [2 1 3]);

%%
load -ASCII x
load -ASCII xnext
load -ASCII u
load -ASCII xdot


%% Orientation: Quat2eul
q_0 = x(:,11)';
q_1 = x(:,12)';
q_2 = x(:,13)';
q_3 = x(:,14)';
eul = [
    atan2(2*(q_0 .* q_1 + q_2 .* q_3),1 - 2*(q_1.^2 + q_2.^2));
    asin(2*(q_0 .* q_2 - q_3 .* q_1));
    atan2(2*(q_0 .* q_3 + q_1 .* q_2),1 - 2*(q_2.^2 + q_3.^2))
    ];
%%
% names = {'Vx', 'Vy', 'Vz', 'Wx', 'Wy', 'Wz', 'W1', 'W2', 'W3', 'W4', 'Q0', 'Q1', 'Q2', 'Q3', 'X', 'Y', 'Z', 'WindX', 'WindY', 'WindZ', 'Gx', 'Gy', 'Gz'};
% figure(2);
% subplot(3,6,1)
% plot(x(:,15))
% title('x')
% subplot(3,6,2)
% plot(x(:,16))
% title('y')
% subplot(3,6,3)
% plot(x(:,17))
% title('z')
% 
% subplot(3,6,7)
% plot(x(:,1))
% title('Vx')
% subplot(3,6,8)
% plot(x(:,2))
% title('Vy')
% subplot(3,6,9)
% plot(x(:,3))
% title('Vz')
% 
% subplot(3,6,4)
% plot(eul(1,:))
% title('Roll')
% subplot(3,6,5)
% plot(eul(2,:))
% title('Pitch')
% subplot(3,6,6)
% plot(eul(3,:))
% title('Yaw')
% 
% subplot(3,6,10)
% plot(x(:,1))
% title('Wx')
% subplot(3,6,11)
% plot(x(:,2))
% title('Wy')
% subplot(3,6,12)
% plot(x(:,3))
% title('Wz')
% 
% subplot(3,6,13:14)
% plot(u)
% title('u')
% legend('1', '2', '3', '4');
% 
% subplot(3,6,15:16)
% plot(x(:,7:10))
% title('W')
% legend('1', '2', '3', '4');
% 
% subplot(3,6,17:18)
% plot(x(:,11:14))
% title('q')
% legend('1', 'i', 'j', 'k', 'Location', 'Best');


%% Controller plot
names = {'Vx', 'Vy', 'Vz', 'Wx', 'Wy', 'Wz', 'W1', 'W2', 'W3', 'W4', 'Q0', 'Q1', 'Q2', 'Q3', 'X', 'Y', 'Z', 'WindX', 'WindY', 'WindZ', 'Gx', 'Gy', 'Gz'};
figure(1);clf;

subplot(2,6,1)
plot(eX(1,:))
title('Vx')
subplot(2,6,2)
plot(eX(2,:))
title('Vy')
subplot(2,6,3)
plot(eX(3,:))
title('Vz')


subplot(2,6,7:9)
plot(eX(7:10,:)'); hold all;
plot(u);
title('W')
% legend('1', '2', '3', '4');

subplot(2,6,4)
plot(eX(11,:))
title('Roll')
subplot(2,6,5)
plot(eX(12,:))
title('Pitch')

subplot(2,6,10)
plot(eX(4,:))
title('Wx')
subplot(2,6,11)
plot(eX(5,:))
title('Wy')
subplot(2,6,12)
plot(eX(6,:))
title('Wz')

%%
% figure(11);
% for i = 1:3
%     subplot(3,1,i);
%     plot(squeeze(A(i,i,:)));
% end

%%
w = 4830;
w = 1;
AA = A(:,:,w);
BB = B(:,:,w);
% XX = x(w,:)';
EX = eX(:,w);
rr = r(:,w);
SS = S(:,:,w);
LL = L(:,:,w);
LR = Lr(:,:,w);
QQ = Q(:,:,w);
RR = R(:,:,w);

AA* EX;
M = zeros(4,13);
M(1:3, 1:3) = 1;
M(4,6) = 1;
% Is the system controllable?
Co=ctrb(AA,BB);

% Number of uncontrollable states
unco=length(AA)-rank(Co)

% Minimal realization
% minsys = minreal(ss(AA,BB,eye(13),0))


% inv(M*((BB*LL-AA)\BB))

%% LQ synthesis
% [K,SM,e] = lqr(AA,BB,QQ,RR,0);

%% System
% sys = ss(A,B,Q,0);
% minsys = minreal(sys)

%% Plot A
% figure(3);
% names = {'Vx', 'Vy', 'Vz', 'Wx', 'Wy', 'Wz', 'W1', 'W2', 'W3', 'W4', 'R', 'P', '1'};
% for i = 1:13
%     for j = 1:13
%         plot(squeeze(A(i, j, :)))
%         title([names{i} '/' names{j}])
%         input('Press enter to continue')
%     end
% end

%%
% figure(9);clf;
% 
% for i = 1:4
%     subplot(4,1,i);
%     plot(u(1:end,i)'); hold all;
%     plot(x(1:end,i+6));
% end

%%
% figure(10);clf;
% for i = 1:4
%     for j = 1:3
%         subplot(3,1,j);
%         plot(eX(2*(i-1)+j,:));
%     end
%     input('Press enter');
% end