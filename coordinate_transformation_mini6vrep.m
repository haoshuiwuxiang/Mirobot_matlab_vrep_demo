
%---ʹ��matlab��vrep���Ϸ��棺��matlab�з��͹ؽڿ���ָ�ʵ��vrep�л�е�۶�Ŀ���ץȡ
disp('Program started');
%---��е����ֵ�ⷽ������⺯�����ɶ�����
MASK = [1 1 1 1 1 1];
%---�ؽڽǶȷ���ֵ��ʼ��
currentJoints = zeros(1,6);
%---ʹ�û����˹�����The Robotics Toolbox for MATLAB (RTB)
%---����DHģ��
%           theta    d       a     alpha     offset
 L1 = Link([    0,0.1128,      0,    0],'modified');%0.11448
 L2 = Link([-pi/2,      0,0.02969,-pi/2],'modified');
 L3 = Link([    0,      0,  0.1076,    0],'modified');
 L4 = Link([    0,0.12867,0.02038,-pi/2],'modified');%0.12867
 L5 = Link([    0,      0,      0, pi/2],'modified');
 L6 = Link([    0,-0.03028,      0, pi/2],'modified');%0.03028

%L2=Link([0, 0  ,  0.115, 0 ],'standard');
%L3=Link([0, 0  ,  0.025, -pi/2 ],'standard');
%L4=Link([0, 0.128, 0 ,pi/2  ],'standard');
%L5=Link([0, 0  ,  0 ,-pi/2  ],'standard');
%L6=Link([0, 0  ,  0 ,0  ],'standard');

s_robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'DOF-6');
%---����vrepԶ�̿�
vrep=remApi('remoteApi');
%---�ȹر�����vrepͨ��
vrep.simxFinish(-1); 
%---����vrep�и��ؽھ������
JointNames={'Joint1','Joint2','Joint3','Joint4','Joint5','Joint6'};
%---����ֵ��Ч���ж�
res2=1;
res3=1;
res4=1;
res5=1;
res7=1;
res8=1;
%---������vrep��ͨ��
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%---�ж�ͨ���Ƿ�ɹ�
if (clientID>-1)   
    disp('Connected to remote API server'); 
    %---����vrep�ĳ�������
    res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait);
    %---��ȡ�����ؽھ��ֵ
     for i = 1:6
         [res, sixJoints(i)] = vrep.simxGetObjectHandle(clientID, ...
             JointNames{i}, vrep.simx_opmode_oneshot_wait);
     end

    %---���vrep����Ӧ���ƵĶ�����ֵ
    %---'target_matlab'ΪץȡĿ�����
    [res,handle_target] = vrep.simxGetObjectHandle(clientID,'target_matlab',vrep.simx_opmode_blocking);
    [res,handle_base_o] = vrep.simxGetObjectHandle(clientID,'Base_o',vrep.simx_opmode_blocking);
   % [res,handle_base] = vrep.simxGetObjectHandle(clientID,'base_frame',vrep.simx_opmode_blocking);
   % [res,handle_table] = vrep.simxGetObjectHandle(clientID,'workshop_frame',vrep.simx_opmode_blocking);
    %---�ж�matlab��vrep�������Ƿ���Ч 
    if(vrep.simxGetConnectionId(clientID) ~= -1) % while v-rep connection is still active
        while (res2==1||res3==1)
            [res2,T_targetPosition]=vrep.simxGetObjectPosition(clientID,handle_target,handle_base_o,vrep.simx_opmode_streaming);
            [res3,T_targetOrientation]=vrep.simxGetObjectOrientation(clientID,handle_target,handle_base_o,vrep.simx_opmode_streaming);
        end
        T_targetPosition
        T_targetOrientation
         %---ת��Ϊ˫���ȸ�����
        T_targetOrientation=double(T_targetOrientation);
        %---ת��ΪT�����е�λ������
        target2baseposition=transl(T_targetPosition);
        %---ת��ΪT�����е���̬
        target2baseOrientation=trotx(T_targetOrientation(1))*troty(T_targetOrientation(2))*trotz(T_targetOrientation(3));%������õ�ԭʼ��̬����RPY��ʾ�ģ� xyz����ת��ŷ����
        %---�õ���������ϵ�ڻ�����ϵ��λ��T
        target2basePos=target2baseposition*target2baseOrientation%��ֻ��λ�õľ����ֻ����̬�ľ���ƴ����������
       
       %---��ȡvrep���ĸ��ؽڵ�ǰ�ĽǶ�ֵ
        for i = 1:6
            [returnCode,currentJoints(i)] = vrep.simxGetJointPosition(clientID, sixJoints(i),...
                    vrep.simx_opmode_oneshot_wait);
            currentJoints(i) = rad2deg(currentJoints(i));
        end
        currentJoints
        
%         g11 =round( target2basePos (1,1),3);%��roundԪ�����ݵ�С�������λ�������е�Ӧ��Ϊ0��������ʾΪ��С������
%         g12 =round( target2basePos (1,2),3);
%         g13 =round( target2basePos (1,3),3);
%         g14 =round( target2basePos (1,4),3);
%         g21 =round( target2basePos (2,1),3);
%         g22 =round( target2basePos (2,2),3);
%         g23 =round( target2basePos (2,3),3);%��roundԪ�����ݵ�С�������λ�������е�Ӧ��Ϊ0��������ʾΪ��С������
%         g24 =round( target2basePos (2,4),3);
%         g31 =round( target2basePos (3,1),3);
%         g32 =round( target2basePos (3,2),3);
%         g33 =round( target2basePos (3,3),3);
%         g34 =round( target2basePos (3,4),3);
%         g41 =round( target2basePos (4,1),3);
%         g42 =round( target2basePos (4,2),3);
%         g43 =round( target2basePos (4,3),3);
%         g44 =round( target2basePos (4,4),3);
%         
%         target2basePos_2 = [g11,g12,g13,g14;
%                             g21,g22,g23,g24;
%                             g31,g32,g33,g34;
%                             g41,g42,g43,g44]
        
        position_cmd_joints=s_robot.ikine(target2basePos,'rlimit',500)
        %---�����ؽڽǶ�ָ����ݷ�����vrep�����?
        vrep.simxPauseCommunication(clientID,1); 
        for i = 1:6
          % res = vrep.simxSetJointPosition(clientID, sixJoints(i),position_cmd_joints(i),vrep.simx_opmode_oneshot);
          res = vrep.simxSetJointTargetPosition(clientID, sixJoints(i),position_cmd_joints(i),vrep.simx_opmode_oneshot);
        end
        vrep.simxPauseCommunication(clientID,0);
        %---�������?
 
    end
    pause(8);
    %---ֹͣvrep�������?
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
   
end
