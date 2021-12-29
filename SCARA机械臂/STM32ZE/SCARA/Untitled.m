	x=input('输入坐标x');
    y=input('输入坐标y');
    L1=input('输入机械臂长度L');
    sigma0=0.5235;
    u=180/pi;
    r=sqrt(x^2+y^2);
	sigma1=acos((r/2)/L1);
	sigma2=atan(x/y);
	sigma3=pi-sigma1-sigma2-sigma0;
	T_sigma1=sigma3*u;
    disp(T_sigma1);
	sigma4=pi-2*sigma1;
	sigma5=sigma1+sigma2;
	if((r^2-2*y*L1)<0||x<0)
        T_sigma2=pi-sigma1-sigma2-sigma4;%L2臂所要转过的角度
         scatter(x,y,'*');
         hold on;
         line([0,L1*sin(sigma5)],[0,L1*cos(sigma5)]);
         line([L1*sin(sigma5),L1*(sin(sigma5)-sin(T_sigma2))],[L1*cos(sigma5),L1*(cos(sigma5)+cos(T_sigma2))]);
    elseif((r^2-2*y*L1)>0&&x>0)
		T_sigma2=sigma1+sigma2+sigma4-pi;
         scatter(x,y,'*');
         hold on;
         line([0,L1*sin(sigma5)],[0,L1*cos(sigma5)]);
         line([L1*sin(sigma5),L1*(sin(sigma5)+sin(T_sigma2))],[L1*cos(sigma5),L1*(cos(sigma5)+cos(T_sigma2))]);
    end
    disp(T_sigma2*u);
    all_step_1=(T_sigma1/360/20)*1600*62;
	all_step_2=(T_sigma2*u/360/20)*1600*62;
    disp('脉冲数为')
     disp(all_step_1);
       disp(all_step_2);
    
  
    
    