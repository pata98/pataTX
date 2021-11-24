%% Data Prep
END = 1000;
x=[1:END];
x1=END-x;
bat=log(x1);
bat(END)=0;
Rand=randn(END);
bat=bat+Rand(1,:);
bat=bat/2;
plot(bat)

%% Mean Filter
bat_mean(1) = bat(1);
for i=2:1000
    bat_mean(i) = (i-1)/i*bat_mean(i-1) + bat(i)/i;
end

plot(x,bat,x,bat_mean)

%% Weighted moving mean filter
alpha = 0.98;
bat_mean = [1:1000];
for i=2:1000
    bat_mean(i) = alpha*bat_mean(i-1) + (1-alpha)*bat(i);
end
plot(x,bat,x,bat_mean,x,log(x1)/2)
title('Alpha: 0.99')
