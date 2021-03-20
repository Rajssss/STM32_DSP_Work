clc;
close all;
clear all;

t=1:0.025:5;
source=5*sin(2*3*t);
noise=5*sin(2*50*3*t);
reference_noise=5*sin(2*50*3*t+ 3/20);
 
primary=source+noise;
 
subplot(5,1,1);
plot(t,source);
ylabel('Source signal');

subplot(5,1,2);
plot(t,noise);
ylabel('Noise signal');
 
 
subplot(5,1,3);
plot(t,reference_noise);
ylabel('Reference noise signal'); 
 
 
 
subplot(5,1,4);
plot(t,primary);
ylabel('Primary signal');
 
order=2;
mu=0.005;
n=length(primary);
delayed=zeros(1,order);
adap=zeros(1,order);
cancelled=zeros(1,n);
for k=1:n
     delayed(1)=reference_noise(k);
     y=delayed*adap';
     cancelled(k)=primary(k)-y;
     adap = adap + 2*mu*cancelled(k)*delayed;
     delayed(2:order)=delayed(1:order-1);
end
 
subplot(5,1,5);
plot(t,cancelled);
ylabel('Output signal');