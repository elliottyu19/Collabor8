clear all
close all

digits = 2:20 ;
for d = digits
    for i = 1:10
        calc_error(i) = double(get_error(d));
    end
    avg_calc_error(d) = max(calc_error);
end

precision = 10.^-(digits);

pred_error = 6*precision*2;

plot(digits,  avg_calc_error(2:20), digits, pred_error(1:19) )

title('End Effector Position Error vs. Precision of Computation')
set(gca, 'YScale', 'log')
xlabel('Number of Precision Digits (based 10)')
ylabel('End Effector Position Error (meters)')
legend('Measured', 'Model')

function total_error = get_error(d)
    R = 2;
    %max_error = 0.5^precision_digits;
    in = [0;0;0];
    
    angles = rand(6,1)*180.0;
   
    H1 = genH('x', angles(1), [0, 0, R/6], 30);
    H2 = genH('x', angles(2), [0, 0, R/6], 30);
    H3 = genH('x', angles(3), [0, 0, R/6], 30);
    H4 = genH('x', angles(4), [0, 0, R/6], 30);
    H5 = genH('x', angles(5), [0, 0, R/6], 30);
    H6 = genH('x', angles(6), [0, 0, R/6], 30);
    
    
    H1e = genH('x', angles(1), [0, 0, R/6], d);
    H2e = genH('x', angles(2), [0, 0, R/6], d);
    H3e = genH('x', angles(3), [0, 0, R/6], d);
    H4e = genH('x', angles(4), [0, 0, R/6], d);
    H5e = genH('x', angles(5), [0, 0, R/6], d);
    H6e = genH('x', angles(6), [0, 0, R/6], d);
    
    He = H2e*H1e;
    He = round(He*10^d)/10^d;
    He = H3e*He;
    He = round(He*10^d)/10^d;
    He = H4e*He;
    He = round(He*10^d)/10^d;
    He = H5e*He;
    He = round(He*10^d)/10^d;
    He = H6e*He;
    He = round(He*10^d)/10^d;
 
    
    out = mulhomvec(H6*H5*H4*H3*H2*H1, in);
    out_error = mulhomvec(He, in);
    out_error = round(out_error*10^d)/10^d;
    total_error = norm(out-out_error);
end

function H = genH(axis, angle, length, d)
    digits(d)
    switch axis
        case 'x'
            rot = rotx(angle);
        case 'y'
            rot = roty(angle);
        case 'z'
            rot = rotz(angle);
    end
    Hrot = rotm2tform(rot);
    Htrans = trvec2tform(length);
    H = vpa(Htrans*Hrot);
end

function out = mulhomvec(H, vec)
     vec(4,1) = 1;
     temp = H*vec;
     out = temp(1:3,1).';
end
