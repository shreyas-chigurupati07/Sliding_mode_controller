function [dX,U,W]=ode_quadrotor(t,X)
    
    % Physical parameters (given)
     m=27*10^-3;
     g=9.8;
     l=46*10^-3;
     Ix=16.5710*10^-6;
     Iy=16.5710*10^-6;
     Iz=29.261652*10^-6;
     Ip=12.65625*10^-8;
     kf=1.28192*10^-8;
     km=5.964552*10^-3;
     Wmax=2618;
     Wmin=0;
    
     omega=0;
    
     % Initializing dX vector
     dX=zeros(12,1);
     X=num2cell(X);
     [x,y,z,phi,theta,psi,dx,dy,dz,dphi,dtheta,dpsi] = deal(X{:});
    
     
    %% Trajectory Equations
    % P0 to P1 
    if (t < 5)
         qp1_d = [0, 0, (2*t.^3)/25 - (3*t.^4)/125 + (6*t.^5)/3125];
         dqp1_d = [0, 0, (6*t.^2)/25 - (12*t.^3)/125 + (6*t.^4)/625];
         ddqp1_d = [0, 0, (12*t)/25 - (36*t.^2)/125 + (24*t.^3)/625];
         x_d = qp1_d(1);
         y_d = qp1_d(2);
         z_d = qp1_d(3);
         dx_d = dqp1_d(1);
         dy_d = dqp1_d(2);
         dz_d = dqp1_d(3);
         ddx_d = ddqp1_d(1);
         ddy_d = ddqp1_d(2);
         ddz_d = ddqp1_d(3);
    
    % P1 to P2 
    elseif (t < 20)
         qp2_d = [(2*t.^3)/675 - t.^4/3375 + (4664065662093477*t.^5/590295810358705651712), 0, 1];
         dqp2_d = [(2*t.^2)/225 - (4*t.^3)/3375 + (23320328310467385*t.^4/590295810358705651712), 0, 0];
         ddqp2_d = [(4*t)/225 - (4*t.^2)/1125 + (23320328310467385*t.^3/147573952589676412928), 0, 0];
         x_d = qp2_d(1);
         y_d = qp2_d(2);
         z_d = qp2_d(3);
         dx_d = dqp2_d(1);
         dy_d = dqp2_d(2);
         dz_d = dqp2_d(3);
         ddx_d = ddqp2_d(1);
         ddy_d = ddqp2_d(2);
         ddz_d = ddqp2_d(3);
    
     % P2 to P3    
     elseif (t < 35)
         qp3_d = [1, (2*t.^3)/675 - t.^4/3375 + (4664065662093477*t.^5)/590295810358705651712, 1];
         dqp3_d = [0, (2*t.^2)/225 - (4*t.^3)/3375 + (23320328310467385*t.^4)/590295810358705651712, 0];
         ddqp3_d = [0, (4*t)/225 - (4*t.^2)/1125 + (23320328310467385*t.^3)/147573952589676412928, 0];
         x_d = qp3_d(1);
         y_d = qp3_d(2);
         z_d = qp3_d(3);
         dx_d = dqp3_d(1);
         dy_d = dqp3_d(2);
         dz_d = dqp3_d(3);
         ddx_d = ddqp3_d(1);
         ddy_d = ddqp3_d(2);
         ddz_d = ddqp3_d(3);
    
    % P3 to P4 
    elseif (t < 50)
         qp4_d = [1 - (2*t.^3)/675 + t.^4/3375 - (4664065662093477*t.^5)/590295810358705651712, 1, 1];
         dqp4_d = [- (2*t.^2)/225 + (4*t.^3)/3375 - (23320328310467385*t.^4)/590295810358705651712, 0, 0];
         ddqp4_d = [- (4*t)/225 + (4*t^2)/1125 - (23320328310467385*t.^3)/147573952589676412928, 0, 0];
         x_d = qp4_d(1);
         y_d = qp4_d(2);
         z_d = qp4_d(3);
         dx_d = dqp4_d(1);
         dy_d = dqp4_d(2);
         dz_d = dqp4_d(3);
         ddx_d = ddqp4_d(1);
         ddy_d = ddqp4_d(2);
         ddz_d = ddqp4_d(3);
    
    % P4 to P5 
    elseif (t <= 65)
         qp5_d = [0, 1 - (2*t.^3)/675 + t.^4/3375 - (4664065662093477*t.^5)/590295810358705651712, 1];
         dqp5_d = [0, - (2*t.^2)/225 + (4*t.^3)/3375 - (23320328310467385*t.^4)/590295810358705651712, 0];
         ddqp5_d = [0, - (4*t)/225 + (4*t.^2)/1125 - (23320328310467385*t.^3)/147573952589676412928, 0];
         x_d = qp5_d(1);
         y_d = qp5_d(2);
         z_d = qp5_d(3);
         dx_d = dqp5_d(1);
         dy_d = dqp5_d(2);
         dz_d = dqp5_d(3);
         ddx_d = ddqp5_d(1);
         ddy_d = ddqp5_d(2);
         ddz_d = ddqp5_d(3);
     end
    
    % Tuning Parameters       % check hint by prof.
    Kp = [80 80]; 
    Kd = [15 18]; 
    K= [13 180 150 25]; 
    lamb=[5 15 13 5]; 
    Phi = 0.9;                % Boundary layer
    
    % Forces realtions (given) to calculate phi_d & theta_d
    Fx = m * (-Kp(1)*(x-x_d) - Kd(1)*(dx-dx_d) + ddx_d);
    Fy = m * (-Kp(2)*(y-y_d) - Kd(2)*(dy-dy_d) + ddy_d);
    
    %% Control Law
    % Control input u1
    e1 = z_d - z;
    de1 = dz_d - dz;
    S1 = de1 + (lamb(1)*e1);           % Sliding surface for u1
    sat1 = min(max(S1/Phi, -1), 1);    % Boundary condition for sliding surface S1
    u1 = (m * (g+ ddz_d + lamb(1)*de1 + K(1)*sat1)) /(cos(phi)*cos(theta));
    
    phi_d = asin(-Fy/u1);
    dphi_d = 0;
    
    % Control input u2
    e2 =  phi - phi_d;
    de2 = dphi - dphi_d;
    S2 = de2 + (lamb(1)*e2);           % Sliding surface for u2
    sat2 = min(max(S2/Phi, -1), 1);    % Boundary condition for sliding surface S2
    u2 = (-dtheta*dpsi*(Iy-Iz)) + (Ip*omega*dtheta) + (((-lamb(2)*de2) - (K(2)*sat2))*Ix);
    
    theta_d = asin(Fx/u1);
    dtheta_d = 0;
    
    % Control input u3
    e3 = theta - theta_d;
    de3 = dtheta - dtheta_d;
    S3 = de3 + (lamb(2)*e3);            % Sliding surface for u3
    sat3 = min(max(S3/Phi, -1), 1);     % Boundary condition for sliding surface S3
    u3 = (-dphi*dpsi*(Iz-Ix)) - (Ip*omega*dphi) - (lamb(3)*dtheta*Iy) - (K(3)*Iy*sat3);
    
    % Control input u4
    psi_d = 0;
    dpsi_d =0;
    e4 = psi - psi_d;
    de4 = dpsi - dpsi_d;
    S4 = de4 + (lamb(3)*e4);          % Sliding surface for u4         
    sat4 = min(max(S4/Phi, -1), 1);   % Boundary condition for sliding surface S4
    u4 = (-dphi*dtheta*(Ix-Iy)) - (lamb(4)*dpsi*Iz) - (K(4)*Iz*sat4);
		    
    u = [u1;u2;u3;u4];
		    
    % Allocation Matrix
    A = [1/(4*kf), -sqrt(2)/(4*kf*l),   -sqrt(2)/(4*kf*l),   -1/(4*km*kf);
	     1/(4*kf), -sqrt(2)/(4*kf*l),	 sqrt(2)/(4*kf*l), 	  1/(4*km*kf);
	     1/(4*kf),  sqrt(2)/(4*kf*l),	 sqrt(2)/(4*kf*l),   -1/(4*km*kf);
	     1/(4*kf),  sqrt(2)/(4*kf*l),   -sqrt(2)/(4*kf*l),    1/(4*km*kf)];
                          
    % Rotor speeds
    W = A * u;
    
    omega = sqrt(W(1)) - sqrt(W(2)) + sqrt(W(3)) - sqrt(W(4));
    
    dX(1)= dx;
    dX(2)= dy;
    dX(3)= dz;
    dX(4)= dphi;
    dX(5)= dtheta;
    dX(6)= dpsi;
    dX(7)= (1/m) * ((cos(phi)*sin(theta)*cos(psi)) + (sin(phi)*sin(psi))) * u1;
    dX(8)= (1/m) * ((cos(phi)*sin(theta)*sin(psi)) - (sin(phi)*cos(psi))) * u1;
    dX(9)= ((1/m) * (cos(phi)*cos(theta)) * u1) - g;
    dX(10)= ((dtheta*dpsi*(Iy-Iz))/Ix) - ((Ip/Ix)*omega*dtheta) + (u2/Ix);
    dX(11)= ((dphi*dpsi*(Iz-Ix))/Iy) + ((Ip/Iy)*omega*dphi) + (u3/Iy);
    dX(12)= ((dphi*dtheta*(Ix-Iy))/Iz) + (u4/Iz);
    
    
end