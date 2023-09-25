function r = QunticTraj(t0,tf,p0,pf)
    syms t
    A = [1 t0 t0^2 t0^3 t0^4 t0^5;
            0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
            0 0 2 6*t0 12*t0^2 20*t0^3;
            1 tf tf^2 tf^3 tf^4 tf^5;
            0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
            0 0 2 6*tf 12*tf^2 20*tf^3];
        
    b = [inv(A)*[p0(1);0;0;pf(1);0;0], inv(A)*[p0(2);0;0;pf(2);0;0], inv(A)*[p0(3);0;0;pf(3);0;0]];
       
    a = [1 t t^2 t^3 t^4 t^5];
    
    r = a*b;
end