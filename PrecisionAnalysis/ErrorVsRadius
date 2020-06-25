clear all
close all

max_error = 5
min_error = 0
num_points = 100
x = min_error:(max_error -min_error)/num_points:max_error;
mun_points = size(x);

for j = 1:length(x) 
    
    for i = 1:6000
        error(i) = get_error(1, x(j));
    end
    
    median_error_data(j) = median(error);
    max_error_data(j) = max(error);
    iqr_error_data(j) = iqr(error);
    
    model(j) = 7*x(j)*1/180;
    
end
plot(x, model, x, max_error_data)
title('End Effector Position Error vs. Robot Arm Length')
xlabel('Robot Arm Length (m)')
ylabel('End Effector Position Error (meters)')
legend('Model', 'Measured')

function total_error = get_error(max_error, R)

    in = [0;0;0];
    
    angles = rand(6,1)*180;
    error = rand(6,1)*max_error;

    H1 = genH('x', angles(1), [0, 0, R/6]);
    H2 = genH('x', angles(2), [0, 0, R/6]);
    H3 = genH('x', angles(3), [0, 0, R/6]);
    H4 = genH('x', angles(4), [0, 0, R/6]);
    H5 = genH('x', angles(5), [0, 0, R/6]);
    H6 = genH('x', angles(6), [0, 0, R/6]);
    
    H1e = genH('x', angles(1)+error(1), [0, 0, R/6]);
    H2e = genH('x', angles(2)+error(2), [0, 0, R/6]);
    H3e = genH('x', angles(3)+error(3), [0, 0, R/6]);
    H4e = genH('x', angles(4)+error(4), [0, 0, R/6]);
    H5e = genH('x', angles(5)+error(5), [0, 0, R/6]);
    H6e = genH('x', angles(6)+error(6), [0, 0, R/6]);
   
    out = mulhomvec(H1*H2*H3*H4*H5*H6, in);
    out_error = mulhomvec(H1e*H2e*H3e*H4e*H5e*H6e, in);
    total_error = norm(out-out_error);
end

function H = genH(axis, angle, length)
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
    H = Htrans*Hrot;
end

function out = mulhomvec(H, vec)
     vec(4,1) = 1;
     temp = H*vec;
     out = temp(1:3,1).';
end
