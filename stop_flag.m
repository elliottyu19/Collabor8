function [stop_flag,dist] = stop_flag(US2, US3, US4, US5, US6, US7, Ph, Uh, Rh, vmax)
     %Define position vectors for start and end points of cylinder of last 5 revolute joints of robot
     d = zeros(length(Ph(1,:)),5);
     U3 = US2(1:3,4);
     S3 = US3(1:3,4);
     R3 = 0.1;
     U4 = S3;
     S4 = US4(1:3,4);
     R4 = 0.1;
     U5 = S4;
     S5 = US5(1:3,4);
     R5 = 0.1;
     U6 = S5;
     S6 = US6(1:3,4);
     R6 = 0.1;
     U7 = S6;
     S7 = US7(1:3,4);
     R7 = 0.1;

     %Calculate distances between each robot arm segment to human
     for i = 1:length(Ph(1,:))
        d(i,1) = dcylinders([U3 S3], R3, [Ph(:,i) Uh(:,i)], Rh(i));
        d(i,2) = dcylinders([U4 S4], R4, [Ph(:,i) Uh(:,i)], Rh(i));
        d(i,3) = dcylinders([U5 S5], R5, [Ph(:,i) Uh(:,i)], Rh(i));
        d(i,4) = dcylinders([U6 S6], R6, [Ph(:,i) Uh(:,i)], Rh(i));
        d(i,5) = dcylinders([U7 S7], R7, [Ph(:,i) Uh(:,i)], Rh(i));
     end
     %Get the Sp value
     S = get_S(vmax);
     
     %Find minimum distance
     dist=min(d,[],2);
     
     
     %Check condition and update stop_flag
     if min(dist) < S
         stop_flag = true;
     else
         stop_flag = false;
     end
end