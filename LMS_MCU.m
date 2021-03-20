function cancelled = LMS_MCU(source, noise, reference_noise)
 
primary=source+noise;

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
 
% subplot(5,1,5);
% plot(t,cancelled);
% ylabel('Output signal');
end