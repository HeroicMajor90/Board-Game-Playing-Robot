function base()
clf;
clear all;
clc
goalXYZ= zeros(3,1);
eStop =zeros(1,8); 
%%%%%%%%%  Initialise myR1 robot
L1 = Link('d',0,'a',2,'alpha',-pi/2,'offset',0,'qlim', [-pi/2,pi/2]);
L2 = Link('d',0,'a',15,'alpha',0,'offset',-pi/1.5,'qlim', [(-1/3*pi),(1/3*pi)]);
L3 = Link('d',0,'a',16,'alpha',0,'offset',+pi/1.5,'qlim', [-pi/3,(2/3*pi)]);
L4 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim', [-2*pi,2*pi]);

uARM = SerialLink([L1 L2 L3 L4], 'name', 'uARM')
uARM.base = transl(0, 0, 0); %Base position uARM

q1 =0; q2 =0; q3 =0; q4 =0; %initialise q states
qa = [q1,q2,q3,q4];
uARM.plot(qa);
hold on;
goalXYZ = [5,15,10];
moveTo(goalXYZ,eStop,qa);
uARM.teach

function iseStop(eStop)
stopped = any(eStop);
 while stopped ~= 0
    pause(2.0);
 end
end

function moveTo(goalXYZ,eStop,qa)
iseStop(eStop);  %check for safety
steps = 15;
%get angles
endEffector = uARM.fkine(qa)
qNow = uARM.ikcon(uARM.fkine(qa)) %current joint angles
%required L1 rotation to get end effector pose
angleGoal = radtodeg(atan((goalXYZ(1,1)/goalXYZ(1,2))))
angleCurrent =radtodeg( atan((endEffector(1,4)/endEffector(2,4))))
angleTotal = angleGoal + angleCurrent

reach = (abs(goalXYZ(1,1))+abs(goalXYZ(1,2))) %of arm gives the amount of reach needed which then determines the
%angle of L3 with regards to the height required.
if reach > 30
    disp('Out of range')
end
if reach < 10
    disp('Out of range')
end
if goalXYZ(1,3) > 19
    disp('Out of range')
end
if goalXYZ(1,3) < 0
    disp('Out of range')
end

%end effector wanted orientation (L3 orientation) 
arg = rotz(angleTotal)%*rotx(16.3-((reach/33)*(goalXYZ(1,3)/25)*16.3)) %(16.3/((goalXYZ(1,3)/6.5)*(reach/10.5)))

%now goalXYZ offset to account for L4 and end effector

goalPoint = [arg(1,1),arg(1,2),arg(1,3),goalXYZ(1,1); arg(2,1),arg(2,2),arg(2,3),goalXYZ(1,2); arg(3,1),arg(3,2),arg(3,3),goalXYZ(1,3);endEffector(4,1),endEffector(4,2),endEffector(4,3),endEffector(4,4);]
qValues = uARM.ikcon(goalPoint) %goal joint values
qMatrix = jtraj(qNow,qValues,steps)
while steps >= 1
            iseStop(eStop);
            uARM.plot(qMatrix((16-steps),:));
            steps = steps - 1;
end 
hold on;
end
function moveIncrement()
end
function isCollision()
end
end