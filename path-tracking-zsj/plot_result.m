clc
clear
sp='4';
%% A
figure(5)
subplot(2,3,1)
load('reference.mat')
plot(refx,refy)
hold on

load('sp10pp.mat')
a=1:size(real_ref,1);
plot(realx(a),realy(a),'--','Color','r')

load('sp10lqrap.mat')
plot(realx(a),realy(a),'-.','Color','g')
load('sp10mpc.mat')
plot(realx(a),realy(a),':','Color','b')

hold off
xlim([0 75])
ylim([-3 60])
xlabel('x (m)')
ylabel('y (m)')
%title('算法跟踪精度坐标对比 speed 4 m/s')
title('轨迹对比,speed 10m/s')
legend('ground','pp','lqr','mpc')
%% b
% figure(6)
subplot(2,3,2)
load('reference.mat')

load('sp10pp.mat')
c=1:size(real_ref);
diffxpp=realx(c)-real_ref(:,1);
diffypp=realy(c)-real_ref(:,2);
load('sp10lqrap.mat')
c=1:size(real_ref);
diffxlqr=realx(c)-real_ref(:,1);
diffylqr=realy(c)-real_ref(:,2);
load('sp10mpc.mat')
c=1:size(real_ref);
diffxmpc=realx(c)-real_ref(:,1);
diffympc=realy(c)-real_ref(:,2);


plot(diffxpp,'--','Color','r')
hold on
plot(diffxlqr,'-.','Color','g')
plot(diffxmpc,':','Color','b')

xlabel('time (s)')
ylabel('magnitude (m)')
title('横向误差')
legend('pp','lqr','mpc')
hold off
%%
subplot(2,3,3)

plot(diffypp,'--','Color','r')
hold on
plot(diffylqr,'-.','Color','g')
plot(diffympc,':','Color','b')

xlabel('time (s)')
ylabel('magnitude (m)')
title('纵向误差')
legend('pp','lqr','mpc')
hold off

%% C cmd
% figure(5)

subplot(2,3,4)
load('reference.mat')

load('sp10pp.mat')
a=1:size(real_ref,1);
plot(realcmd(a),'--','Color','r')
hold on
load('sp10lqrap.mat')
plot(realcmd(a),'-.','Color','g')
load('sp10mpc.mat')
plot(realcmd(a),':','Color','b')

hold off
%xlim([0 80])
%ylim([-20 20])
xlabel('time (s)')
ylabel('steer cmd (degree)')
title('算法跟踪下发角度')
legend('pp','lqr','mpc')

%% d dcmd/dx difference cmd
subplot(2,3,5)
load('reference.mat')
%plot(refx,refy)


load('sp10pp.mat')
a=1:size(real_ref,1);
b=size(real_ref,1);
T=2;
for i=1:b-T-2
    kpp(i)=(realcmd(i+T)-realcmd(i))/(a(i+T)-a(i));%k=dy/dx=Δy/Δx
end
plot(kpp,'--','Color','r')

hold on
load('sp10lqrap.mat')
for i=1:b-T-2
    klqr(i)=(realcmd(i+T)-realcmd(i))/(a(i+T)-a(i));%k=dy/dx=Δy/Δx
end
plot(klqr,'-.','Color','g')

load('sp10mpc.mat')
for i=1:b-T-2
    kmpc(i)=(realcmd(i+T)-realcmd(i))/(a(i+T)-a(i));%k=dy/dx=Δy/Δx
end
plot(kmpc,':','Color','b')
hold off
xlim([0 180])
%ylim([-20 20])
xlabel('time (s)')
ylabel('gradient (degree/s)')
title('下发角度斜率')
legend('pp','lqr','mpc')

%% E time cost

subplot(2,3,6)
load('reference.mat')
%plot(refx,refy)


load('sp10pp.mat')
a=1:size(real_ref,1);
plot(real_t(a),'--','Color','r')
sumpp=sum(real_t);
hold on
load('sp10lqrap.mat')
plot(real_t(a),'-.','Color','g')
sumlqr=sum(real_t);
load('sp10mpc.mat')
plot(real_t(a),':','Color','b')
summpc=sum(real_t);
hold off
%xlim([0 80])
xlim([0 180])
%xlabel('step ()')
ylabel('time cost (s)')
xlabel('time (s)')
title(['算法耗时 ','pp:',num2str(sumpp),', lqr:',...
    num2str(sumlqr),', mpc:',num2str(summpc),...
   ])
legend('pp','lqr','mpc')


%% F wave affect






