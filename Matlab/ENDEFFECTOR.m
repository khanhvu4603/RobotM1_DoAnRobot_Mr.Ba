function P_EE = ENDEFFECTOR(d,the1,the2,L1,L2,L3)
 P_EE=[L1+L3*cos(the1+the2)+L2*cos(the1);
       L3*sin(the1+the2)+L2*sin(the1);
        d;];
end
