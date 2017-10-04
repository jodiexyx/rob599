function newapple = apple_transformations(apple_filepath)
    affine = eye(3);
    %1. Translate the apple by 250 pixels along the x axis and 200 along the y axis in the currentframe.
    affine = affine * [1,0,250;0,1,200;0,0,1];
    %2. Rotate the apple about the current frame z axis 90 degrees clockwise.
    affine = affine * [cos(0.5*pi),sin(0.5*pi),0;-sin(0.5*pi),cos(0.5*pi),0;0,0,1];
    %3. Translate the apple by 100 pixels along the current frame y axis.
    affine = affine * [1,0,0;0,1,100;0,0,1];
    %4. Rotate the apple about the world frame z axis 45 degrees counterclockwise.
    affine = [cos(-0.25*pi),sin(-0.25*pi),0;-sin(-0.25*pi),cos(-0.25*pi),0;0,0,1] * affine;
    %5. Scale the apple by a factor of 2 in the current frame.
    affine = affine * [2,0,0;0,2,0;0,0,1]; 
    %6. Translate the apple by -150 pixels along the world frame y axis.
    affine = [1,0,0;0,1,-150;0,0,1] * affine;
    % use provided load_apple function
    apple = load_apple(apple_filepath);
    newapple = uint8(zeros(500));
    for i=0:499,
        for j=0:499,
            old_coors = inv(affine)*[i;j;1];
            oldi=round(old_coors(1));
            oldj=round(old_coors(2));
            if oldi>=0 && oldi<=99 && oldj>=0 && oldj<=99,
                newapple(j+1,i+1)=apple(oldj+1,oldi+1);
            end
        end
    end
end