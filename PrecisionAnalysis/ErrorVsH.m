clear all
close all

maxHerror = [0,0,0,0,0,0];

N = 50;
x = 1:N;
for j = 1:15000
    data(j,:) = get_error(N);
end

for i = 1:N
    max_error(i) = max(data(:,i));
end

model = (x)*1/360*2

plot(x, max_error, x, model)
title('End Effector Position Error vs. Degrees of Freedom')
xlabel('Number of Degrees of Freedom (DOF)')
ylabel('End Effector Position Error (meters)')
legend('Measured', 'Model')

function Herror = get_error(N)
    max_error = 1;
    R = 2;
    in = [0;0;0];

    angles = rand(N,1)*360;
    error = rand(N,1)*max_error;
    
    for i =1:N
        H(:,:,i) = genH('x', angles(i), [0, 0, R/N]);
        He(:,:,i) = genH('x', angles(i)+error(i), [0, 0, R/N]);
    end
    
    Hf(:,:,1) = H(:,:,1);
    Hfe(:,:,1) = He(:,:,1);
    pos = mulhomvec(Hf(:,:,1), in);
    pose = mulhomvec(Hfe(:,:,1), in);
    Herror(1) = norm(pos - pose);
    
    for i =2:N
        Hf(:,:,i) = Hf(:,:,i-1)*H(:,:,i);
        Hfe(:,:,i) = Hfe(:,:,i-1)*He(:,:,i);
        pos = mulhomvec(Hf(:,:,i), in);
        pose = mulhomvec(Hfe(:,:,i), in);
        Herror(i) = norm(pos - pose);

    end
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
