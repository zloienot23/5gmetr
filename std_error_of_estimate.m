clear all
close all
clc

for j=1:1:100
f0 = 465; %465 кГЦ
fs = 1000; %1 МГц
T = 1/fs;

t = 0:T:10-T; % время накопления сигнала и индикации

%rng(40);
num = randi(3); %число найденных сигналов

x = nan(num,length(t));

n=50*num; %СКО шума наблюдения
noise = zeros(num,length(t));


%создадим модель шума
for i=1:1:length(t)
    noise(:,i) = normrnd(0,n,[num,1]);
end


%rng(26);
for i=1:1:num
    A=randi(1000);
    A=1.1*A;
    x(i,:) = A*sin(2*pi*f0*t);
end


Et1 = 0; %начальная энергия референсного сигнала
Et2 = 0;
Et3 = 0;
for i=1:1:length(t)
    if num==1
        Et1 = Et1 + T*(x(1,i)^2);
    elseif num==2
        Et1 = Et1 + T*(x(1,i)^2);
        Et2 = Et2 + T*(x(2,i)^2);
    else
        Et1 = Et1 + T*(x(1,i)^2);
        Et2 = Et2 + T*(x(2,i)^2);
        Et3 = Et3 + T*(x(3,i)^2);
    end
end

E_ref = (Et1/3600)*1e6+(Et2/3600)*1e6+(Et3/3600)*1e6; %общая энергия референсных сигналов
E_ref = E_ref/(4*pi*25)/1e4; % энергетическая эскпозиция

y = x+noise;
Et_y1 = 0; %начальная энергия смеси сигнал/шум
Et_y2 = 0;
Et_y3 = 0;
for i=1:1:length(t)
    if num==1
        Et_y1 = Et_y1 + T*(y(1,i)^2);
    elseif num==2
        Et_y1 = Et_y1 + T*(y(1,i)^2);
        Et_y2 = Et_y2 + T*(y(2,i)^2);
    else
        Et_y1 = Et_y1 + T*(y(1,i)^2);
        Et_y2 = Et_y2 + T*(y(2,i)^2);
        Et_y3 = Et_y3 + T*(y(3,i)^2);
    end
end

E_noise = (Et_y1/3600)*1e6+(Et_y2/3600)*1e6+(Et_y3/3600)*1e6; %энергия смеси сигнал/шум
E_noise = E_noise/(4*pi*25)/1e4; % энергетическая эскпозиция



%применение фильтра
if num==1
        u1=wiener_filter(x(1,:),y(1,:),t,n); 
    elseif num==2
        u1=wiener_filter(x(1,:),y(1,:),t,n); 
        u2 = wiener_filter(x(2,:),y(2,:),t,n); 
    else
        u1=wiener_filter(x(1,:),y(1,:),t,n); 
        u2 = wiener_filter(x(2,:),y(2,:),t,n); 
        u3 = wiener_filter(x(3,:),y(3,:),t,n); 
end


Et_u1 = 0; %начальная энергия оцененного сигнала
Et_u2 = 0;
Et_u3 = 0;
for i=1:1:length(t)
    if num==1
        Et_u1 = Et_u1 + T*(u1(1,i)^2);
    elseif num==2
        Et_u1 = Et_u1 + T*(u1(1,i)^2);
        Et_u2 = Et_u2 + T*(u2(1,i)^2);
    else
        Et_u1 = Et_u1 + T*(u1(1,i)^2);
        Et_u2 = Et_u2 + T*(u2(1,i)^2);
        Et_u3 = Et_u3 + T*(u3(1,i)^2);
    end
    
end
E_u = (Et_u1/3600)*1e6+(Et_u2/3600)*1e6+(Et_u3/3600)*1e6; %общая энергия оцененных сигналов
E_u = E_u/(4*pi*25)/1e4; % энергетическая эскпозиция

massiv_E(1,i) = E_ref;
massiv_E(2,i) = E_noise;
massiv_E(3,i) = E_u;
disp(j)
end

error = std(massiv_E(1,:)-massiv_E(3,:)); %СКО ошибки оценки энергетической экспозиции









