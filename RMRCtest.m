clear all
UR3e = UR3e(transl(0, 0, 0));
UR3e_Pose = UR3e.model.fkine(UR3e.model.getpos()).T;
steps = 50;

M = [1 1 zeros(1,4)];   

T1_x = UR3e_Pose(1,4);
T1_y = UR3e_Pose(2,4);

disp('Enter the ending pose as x and y values:');
T2_x = input('T2_x = ');
T2_y = input('T2_y = ');


x1 = [T1_x T1_y]';
x2 = [T2_x T2_y]';
deltaT = 0.05;                                     


x = zeros(2,steps);
s = lspb(0,1,steps);                                
for i = 1:steps
    x(:,i) = x1*(1-s(i)) + s(i)*x2;        
end


qMatrix = nan(steps,2);

qMatrix(1,:) = UR3e.model.ikine(UR3e_Pose, 'q0', [0 0], 'mask', M);               


for i = 1:steps-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                       
    J = UR3e.jacob0(qMatrix(i,:));            
    J = J(1:2,:);                           
    qdot = inv(J)*xdot;                      
    qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                  
end


UR3e.plot(qMatrix,'trail','r-');