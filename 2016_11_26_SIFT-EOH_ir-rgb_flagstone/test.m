f_l = 3.9676;        % left camera is ir
f_r = 7.8545;         % right camera is RGB
b =  3*25.4;         % stereo baseline
d =  78.4954;         % dual focal length baseline
verticalAlignmentOffset = 16;    % this is the vertical offset of center rows for the two cameras
pixelDimL = .006;
pixelDimR = .006;
Zmin = 600;
Zmax = 3000;

P1 = frames1EOHSIFT(1:2,:);
P2 = frames2EOHSIFT(1:2,:);
matches = matchesEOHSIFT;
[M1,N1,K1]=size(I1) ;
[M2,N2,K2]=size(I2) ;

stack = 'h'

switch stack
  case 'h'
    N3=N1+N2 ;
    M3=max(M1,M2) ;
    oj=N1 ;
    oi=0 ;
  case 'v'
    M3=M1+M2 ;
    N3=max(N1,N2) ;
    oj=0 ;
    oi=M1 ;
  case 'd'
    M3=M1+M2 ;
    N3=N1+N2 ;
    oj=N1 ;
    oi=M1 ;
  case 'o'
    M3=max(M1,M2) ;
    N3=max(N1,N2) ;
    oj=0;
    oi=0;
  otherwise
    error(['Unkown stacking type '''], stack, ['''.']) ;
end

K = size(matches, 2) ;
nans = NaN * ones(1,K) ;

%IR Image
x = [ P1(1,matches(1,:)); P2(1,matches(2,:)); nans ];
y = [ P1(2,matches(1,:)); P2(2,matches(2,:)); nans ];
imshow(I1)
hold on
plot(x(1,1),y(1,1),'g*')
hold off
x_l = (x(1,1) - 640/2)*(.0048)*(.0048/.006)

%vl image
figure
imshow(I2)
hold on 
plot(x(2,1),y(2,1),'g*')
hold off

ir_filename  = 'images/ir01_rect.tif';
I1b=imreadbw(ir_filename);
[m,n] = size(I1b);
X = (n-640)/2;
Y = (m-480)/2;
I1b = imcrop(I1b,[X,Y,639,479]);
figure()
imshow(I1b)
hold on
X = x(1,1);
Y = y(1,1);
X = (X - 640/2)*(4.8/6) + 640/2;
Y = (Y - 480/2)*(4.8/6) + 480/2;
plot(X,Y,'g*')
hold off
x_l = (X - 640/2)*.0048

pixel_r = x(1,1);               
pixel_l = x(2,1);
x_l = (pixel_l - 640/2)*(.0048)*(.0048/.006);
x_r = (pixel_r - 640/2)*pixelDimR;
Z = (x_l*f_r*d+f_r*f_l*b)/(f_l*x_r - f_r*x_l);
X_r = x_r*Z/f_r;
X_l = X_r+b;
x_l = f_l*(X_l+b)/(Z+d);

pixel_l = x(1,1);               
pixel_r = x(2,1);
x_l = (pixel_l - 640/2)*(.0048)*(.0048/.006);
x_r = (pixel_r - 640/2)*pixelDimR;
Z = (x_l*f_r*d-f_r*f_l*b)/(f_l*x_r - f_r*x_l);
X_r = x_r*Z/f_r;
X_l = X_r+b;
x_l = f_l*(X_l)/(Z+d);




