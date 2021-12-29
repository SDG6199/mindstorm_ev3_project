close all; clear; clc

addpath('/home/pc/catkin_ws/src/mavros/matlab_msg_gen_ros1/glnxa64/install/m')
addpath('/home/pc/gui')
addpath('/home/pc/Downloads/project code')
%folderpath=fullfile('~/px4_ws/src/mavros');
%rosgenmsg(folderpath);

%rosinit
desiredRate= 5;
rate = rateControl(desiredRate);
rate.OverrunAction = 'slip';

global setpub; global setmsg; global trans; global x; global y; global z; global state; global odomsub; global imusub;
trans=[5;20;2];
odomsub =rossubscriber('/mavros/local_position/odom','nav_msgs/Odometry');
imusub =rossubscriber('/mavros/imu/data','sensor_msgs/Imu');
depthsub =rossubscriber('/camera/depth/image_raw','sensor_msgs/Image');
imagesub =rossubscriber('/camera/rgb/image_raw','sensor_msgs/Image');

[setpub,setmsg] = rospublisher('/mavros/setpoint_position/local','geometry_msgs/PoseStamped');
K=([565.6008952774197, 0.0, 320.5; 0.0, 565.6008952774197, 240.5; 0.0, 0.0, 1.0]);

for i=1:15
    setmsg.Pose.Position.X = 10;
    setmsg.Pose.Position.Y = 20;
    setmsg.Pose.Position.Z = 2;
    send(setpub,setmsg);
    pause(0.1);
       
    if rem(i,5)==0
        arming = rossvcclient('mavros/cmd/arming');
        testreq1 = rosmessage(arming);
        testreq1.Value=1;
        response1 = call(arming,testreq1,'Timeout',2);
        if testreq1.Value==1
            disp('Arming enabled');
        else
           disp('Arming failed');
        end

        set_mode = rossvcclient('mavros/set_mode');
        testreq2 = rosmessage(set_mode);
        testreq2.CustomMode='OFFBOARD';
        response2 = call(set_mode,testreq2,'Timeout',2);
        if testreq2.CustomMode=='OFFBOARD'
            disp('Offboard enabled');
        else
           disp('Offboard failed');
        end
    end
end

reset(rate);
X=[];
Y=[];
Z=[];
VX=[];
VY=[];
VZ=[];

gui1(); gui2();
handles1 = guidata(gui2);
handles2 = guidata(gui1);

pretrained = load('tinyYOLOv2-coco.mat');
detector = pretrained.yolov2Detector;

inputSize = detector.TrainingImageSize;

%% rrt initialize
flag=1;
goal_i=1;
goal2_i=1;
xd=[10;20]; 
map_size=[60,60];
way1_goal=set_orient_by_map(map_size,[1,4]); 
way2_goal=[0;0];
% 맵의 크기에 따라 set. 각도까지. 3번째 인자는 맵의 크기를 나눌 수. 맵의 크기를 나눌 W수(2)는 짝수여야 함. 
x_goal=way1_goal(1:2,:);
quat_goal=way1_goal(3:6,:);
quat_goal2=[0 0 0 0];
step_x=6;
step_y=6;
step_id=1;
yolo_range=7.5;
flag_truck=1;
xdz=2;
id=1;
m=1;
map=binaryOccupancyMap(map_size(1)+10,map_size(2)+10,10);
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss); 
sv.ValidationDistance = 0.01; 
%%

i=1;
j=1;
prev_time=0;
update_cycle=9;
global_path_flag=1;
while 1
    time = rate.TotalElapsedTime;
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    if time-prev_time>update_cycle
        flag=1;
        disp('flag on')
        prev_time=time;
    end
    global state;
    state = receive(odomsub);
    imu = receive(imusub);
    
    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
    Rot=quat2rotm(quat);
    image_msg=receive(imagesub);
    depth_msg=receive(depthsub);

    [img,im_alpha] = readImage(image_msg);
    [dep,dp_alpha] = readImage(depth_msg);
    dp= imresize(dep,inputSize);
    sz= size(img);
    if numel(img)==sz(1)*sz(2) 
        Image = cat(3, img, img, img);
        im = imresize(Image,inputSize);
    else
        im = imresize(img,inputSize);
    end
    %% yolo
    [boxes,scores,labels] = detect(detector,im);
    if ~isempty(boxes)  
        label=char(labels);
        score=num2str(scores);
        [depth_value,obj_pos]=Depth_extract(boxes,dp);
        label_str={};
        if min(depth_value)<yolo_range
            for ii=1:size(boxes,1) 
                depth_val=num2str(depth_value(ii));
                label_str{ii}=[label(ii,:) ' : ' score(ii,:) ', depth : ' depth_val];
            end
            label_to_im=label_str';
            im = insertObjectAnnotation(im,'rectangle',boxes,label_to_im);
            index=find(depth_value==min(depth_value));

            if strcmp(label(index,:),'truck') & flag_truck==1
                c_truck_pos_x=obj_pos(index,1)-K(1,3)+0.5;
                c_truck_pos_y=obj_pos(index,2)-K(2,3)+0.5;

                c_truck_xDf=double(depth_value(index)/(K(1,1)));
                c_truck_yDf=double(depth_value(index)/(K(2,2)));

                c_obj_x=min(depth_value)+trans(1);
                c_obj_y=c_truck_pos_x*c_truck_xDf+trans(2);
                c_obj_z=c_truck_pos_y*c_truck_yDf+trans(3);
                truck_pos=[c_obj_x; c_obj_y; 1];
                way2_goal=set_orient_by_object(40,yolo_range,truck_pos(1),truck_pos(2));
                global_path_flag=2;
                flag_truck=0;
            else
                disp("")
            end

            if  strcmp(label(index,:),'person') & global_path_flag==2
                time = rate.TotalElapsedTime;
                fprintf("find person!! \n")
                X=['Iteration No.: ',num2str(i), ', Arrival time: ', num2str(time),'s'];
                disp(X);
                break;
            end
        end
    end
    x=state.Pose.Pose.Position.X;
    X=[X;x];
    y=state.Pose.Pose.Position.Y;
    Y=[Y;y];
    z=state.Pose.Pose.Position.Z;
    Z=[Z;z];
    vx=state.Twist.Twist.Linear.X;
    VX=[VX;vx];
    vy=state.Twist.Twist.Linear.Y;
    VY=[VY;vy];
    vz=state.Twist.Twist.Linear.Z;
    VZ=[VZ;vz];
    
    global trans; global x; global y; global z; 
    trans=[x;y;z];

    set(handles2.datax, 'String', x);
    set(handles2.datay, 'String', y);
    set(handles2.dataz, 'String', z);
    set(handles2.datavx, 'String', vx);
    set(handles2.datavy, 'String', vy);
    set(handles2.datavz, 'String', vz);
    
    scale=1/1;
    re_img=imresize(img,scale);
    re_dep=imresize(dep,scale);

    cloud=[];
    dep_mm=re_dep*1000;
    Sd=size(dep_mm);
    [pX, pY]=meshgrid(1:Sd(2),1:Sd(1));
    
    pX=pX-K(1,3)*scale+0.5;
    pY=pY-K(2,3)*scale+0.5;

    xDf=double(dep_mm/(K(1,1)*scale));
    yDf=double(dep_mm/(K(2,2)*scale));
    
    pX=pX.*xDf;
    pY=pY.*yDf;

    
    pXY=cat(3,pX,pY);
    
    cloud_nan=cat(3,pXY,dep_mm);
    cloud_nan=reshape(cloud_nan,[],3)/1000;
    cloud = rmmissing(cloud_nan);

    n=length(cloud);
    cam_eul=[pi/2+pi 0 pi/2+pi];
    rotcam=eul2rotm(cam_eul);

    cloud_affine=[];
    cloud_affine=([Rot trans]*[rotcam zeros(3,1);0 0 0 1]*[cloud';ones(1,n)])';
    
    ptCloud=pointCloud(cloud_affine);
    ptCloud_d=pcdownsample(ptCloud,'gridAverage',0.1);
    
    [groundPtsIdx,nonGroundPtCloud,groundPtCloud] = segmentGroundSMRF(ptCloud_d);

    ptCloud_obs = rmmissing(nonGroundPtCloud.Location);
    
    minDistance=0.5;
    minPoints=10;
    [labels,numClusters] = pcsegdist(nonGroundPtCloud,minDistance,'NumClusterPoints',minPoints);

    idxValidPoints = find(labels);
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(nonGroundPtCloud,idxValidPoints);

    numobs=double(numClusters);
    obs=zeros(2,numobs);
    offset=10;
    if z>0.5 & numClusters~=0
        for k=1:numobs
            temp=select(segmentedPtCloud,labelColorIndex==k);
            obs(1,k)=mean(temp.Location(:,1))+offset;
            obs(2,k)=mean(temp.Location(:,2))+offset;
        end
        map=binaryOccupancyMap(map_size(1)+10,map_size(2)+10,10);
        setOccupancy(map,obs',ones(size(obs',1),1));
        inflate(map,1.5);  % @ inflate
        
        if flag==1
            xd=[];
            %%
            sv.Map = map;
            %ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
            if global_path_flag==1
                planner_rad_coef=0.2;             
                planner = plannerHybridAStar(sv,'MinTurningRadius',4*planner_rad_coef,'MotionPrimitiveLength',6*planner_rad_coef,'InterpolationDistance',2);   % @ interpolate. related to velocity.
            else
                planner_rad_coef=0.3; 
                if goal2_i==3
                    planner_rad_coef=0.5%1
                end
                planner = plannerHybridAStar(sv,'MinTurningRadius',4*planner_rad_coef,'MotionPrimitiveLength',6*planner_rad_coef,'InterpolationDistance',2);   % @ interpolate. related to velocity.

            end


            switch global_path_flag
                case 1              %% global_path_flag=1
                    if x_goal(1,goal_i)-trans(1)>0                    %+x방향
                        if x_goal(1,goal_i)-trans(1)>step_x
                            t_x_goal=[trans(1)+step_x; x_goal(2,goal_i)];
                            while check_min_dist(obs-offset,t_x_goal)<1.5+0.5    % 장애물과 너무 가까우면
                                t_x_goal=[t_x_goal(1)+0.2; t_x_goal(2)];
                                disp('corrected goal')
                            end
                        else
                            t_x_goal=x_goal(:,goal_i);
                            while check_min_dist(obs-offset,t_x_goal)<1.5+0.5    % 장애물과 너무 가까우면
                                t_x_goal=[t_x_goal(1)+0.2; t_x_goal(2)];
                                disp('corrected goal')
                            end
                        end
                    elseif x_goal(1,goal_i)-trans(1)<0                %-x방향
                        if trans(1)-x_goal(1,goal_i)>step_x
                            t_x_goal=[trans(1)-step_x; x_goal(2,goal_i)];
                            while check_min_dist(obs-offset,t_x_goal)<1.5+0.5    % 장애물과 너무 가까우면
                                t_x_goal=[t_x_goal(1)-0.2; t_x_goal(2)];
                                disp('corrected goal')
                            end
                        else
                            t_x_goal=x_goal(:,goal_i);
                            while check_min_dist(obs-offset,t_x_goal)<1.5+0.5    % 장애물과 너무 가까우면
                                t_x_goal=[t_x_goal(1)-0.2; t_x_goal(2)];
                                disp('corrected goal')
                            end
                        end  %'
                    end
                    if goal_i>1
                        if x_goal(2,goal_i)-x_goal(2,goal_i-1)~=0     %+y방향
                            if x_goal(2,goal_i)-trans(2)>step_y
                                t_x_goal=[x_goal(1,goal_i); trans(2)+step_y];
                                while check_min_dist(obs-offset,t_x_goal)<1.5+0.5    % 장애물과 너무 가까우면
                                    t_x_goal=[t_x_goal(1); t_x_goal(2)+0.2];
                                    disp('corrected goal')
                                end
                            else
                                t_x_goal=x_goal(:,goal_i);
                                while check_min_dist(obs-offset,t_x_goal)<1.5+0.5    % 장애물과 너무 가까우면
                                    t_x_goal=[t_x_goal(1); t_x_goal(2)+0.2];
                                    disp('corrected goal')
                                end
                            end
                        end
                    end   
                case 2    %% global_path_flag=2
                    x_goal2=way2_goal; 
                    t_x_goal=x_goal2(:,goal2_i);
                    prev_time=time;
                    update_cycle=30;
                    if goal2_i==1
                        [quat_goal2,ret2]=calculate_orient(trans(1:2),x_goal2(:,1));
                    end
                otherwise
                    disp("")
            end
            start = [trans(1)+offset trans(2)+offset 0];
            temp_goal=[trans(1); trans(2); 2];
            while check_min_dist(obs-offset,trans(1:2))<1.5+0.5    % 장애물과 너무 가까우면
                global trans; global x; global y; global z; global state; global odomsub;
                state=receive(odomsub);
                trans=[state.Pose.Pose.Position.X;state.Pose.Pose.Position.Y;state.Pose.Pose.Position.Z;];
                temp_goal = [temp_goal(1)-0.8; temp_goal(2); 2]; 
                disp('correct start')
                while norm(trans(1:2)-temp_goal(1:2))>0.5     %충분히 가까워질때까지 반복
                    global setpub; global setmsg; global trans; global x; global y; global z; global state; global odomsub;
                    state=receive(odomsub);
                    trans=[state.Pose.Pose.Position.X; state.Pose.Pose.Position.Y; state.Pose.Pose.Position.Z];
                    setmsg.Pose.Position.X = temp_goal(1);
                    setmsg.Pose.Position.Y = temp_goal(2);
                    setmsg.Pose.Position.Z = temp_goal(3);
                    send(setpub,setmsg);
                    fprintf("waiting correct start.. %f \n", trans(1))
                    start = [trans(1)+offset trans(2)+offset 0];
                end
            end
            goal = [t_x_goal(1)+offset t_x_goal(2)+offset 0]
            set(handles2.Start_x, 'String', start(1)-offset);
            set(handles2.Start_y, 'String', start(2)-offset);
            set(handles2.Goal_x, 'String', goal(1)-offset);
            set(handles2.Goal_y, 'String', goal(2)-offset);
            tic
            refpath = plan(planner,start,goal);
            toc
            id=1;
            xd(1,:)=(refpath.States(:,1))'-offset;
            xd(2,:)=(refpath.States(:,2))'-offset; 
            fprintf("xd is updated \n");
            flag=0;
        end
    end

    %%
    plot3(X,Y,Z,'LineWidth',2,'Color', 'b','parent',handles1.axes1); hold( handles1.axes1, 'on' )
    plot3(xd(1,:),xd(2,:),xdz*ones(size(xd(2,:))),'LineWidth',2,'Color', 'r','parent',handles1.axes1); 
    plot3(X,Y,Z,'LineWidth',2,'Color', 'b','parent',handles2.axes3); hold( handles2.axes3, 'on' )
    plot3(xd(1,:),xd(2,:),xdz*ones(size(xd(2,:))),'LineWidth',2,'Color', 'r','parent',handles2.axes3); 
    
    plot(xd(1,:),xd(2,:),'LineWidth',2,'Color', 'r','parent',handles2.axes4);  hold( handles2.axes4, 'on' )
    plot(obs(1,:),obs(2,:),'ok','MarkerSize',2,'parent',handles2.axes4);
    hold( handles2.axes4, 'off' );    grid(handles2.axes4,'on');

    axis(handles2.axes4,[-10 map_size(1) -10 map_size(2)]);
    if numClusters~=0
        plot3(segmentedPtCloud.Location(:,1),segmentedPtCloud.Location(:,2),segmentedPtCloud.Location(:,3),'ok','MarkerSize',1,'parent',handles1.axes1); 
        plot3(segmentedPtCloud.Location(:,1),segmentedPtCloud.Location(:,2),segmentedPtCloud.Location(:,3),'ok','MarkerSize',1,'parent',handles2.axes3); 

    else
        plot3(ptCloud_d.Location(:,1),ptCloud_d.Location(:,2),ptCloud_d.Location(:,3),'ok','MarkerSize',1,'parent',handles1.axes1); 
        plot3(ptCloud_d.Location(:,1),ptCloud_d.Location(:,2),ptCloud_d.Location(:,3),'ok','MarkerSize',1,'parent',handles2.axes3); 

    end
    
    hold(handles1.axes1, 'off' );    grid(handles1.axes1,'on');
    axis(handles1.axes1,[-10 map_size(1)+10 -10 map_size(2)+10 -1 3]);
    xlabel(handles1.axes1,'x');
    ylabel(handles1.axes1,'y');
    zlabel(handles1.axes1,'z');

    hold(handles2.axes3, 'off' );    grid(handles2.axes3,'on');
    x_range_min=-2+trans(1); x_range_max=10+trans(1); y_range_min=-4+trans(2); y_range_max=4+trans(2); 

    axis(handles2.axes3,[x_range_min x_range_max y_range_min y_range_max -1 3]);

    xlabel(handles2.axes3,'x');
    ylabel(handles2.axes3,'y');
    zlabel(handles2.axes3,'z');

    im = imresize(im,sz(1:2));
    imshow(im,'Parent',handles2.axes1);
    show(map,'Parent',handles2.axes2);  hold(handles2.axes2,'on' ); 
    
    pX=[];
    pY=[];
    pZ=[];

    setmsg.Pose.Position.X = xd(1,id);
    setmsg.Pose.Position.Y = xd(2,id);
    setmsg.Pose.Position.Z = xdz;
    if global_path_flag==1
        setmsg_orient(quat_goal(:,goal_i));
    else
        setmsg_orient(quat_goal2'); %'
    end

    send(setpub,setmsg);
    
    if (norm(trans(1:2)-xd(:,id))<1 & z>0.5)  
        if id<size(xd,2)-(step_id-1)
            fprintf('id: %d\n',id)
            id=id+step_id;   % step_id는 경로를 불연속적으로 추종할 간격.
        end
        if (norm(trans(1:2)-x_goal(1:2,goal_i))<3 & global_path_flag==1)  
            goal_i=goal_i+1;
            switch mod(goal_i,4)    % +x축은 quat로 보낼땐 180도, eul로 받을땐 0.
                case 2    % +y방향 회전
                    global imusub;
                    imu = receive(imusub);
                    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
                    eul=quat2eul(quat);
                    while 1      %@ 돌때까지 Rot를 보고 판단하며 기다린다.
                        global setpub; global setmsg;
                        setmsg_orient(quat_goal(:,goal_i));
                        send(setpub,setmsg);
                        global imusub;
                        imu = receive(imusub);
                        quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
                        eul=quat2eul(quat);
                        fprintf("waiting rotate to 90.. %f \n", eul(1))
                            if abs(eul(1)-90*pi/180)<0.1
                                fprintf("finish turn \n")
                                break;
                            end
                    end
                    flag=1; 
                    goal_i=goal_i+1;
                    step_id=3;
                    update_cycle=9;
                case 0      % -x방향 회전      
                    if mod(j,2)==1
                        global imusub;
                        imu = receive(imusub);
                        quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
                        eul=quat2eul(quat);
                        
                        while 1      %@ 돌때까지 Rot를 보고 판단하며 기다린다.
                            global setpub; global setmsg;
                            setmsg_orient(quat_goal(:,goal_i));
                            send(setpub,setmsg);
                            global imusub;
                            imu = receive(imusub);
                            quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
                            eul=quat2eul(quat);
                            fprintf("waiting rotate to 180.. %f \n", eul(1))
                            if abs(eul(1)-180*pi/180)<0.1
                                fprintf("finish turn \n")
                                break;
                            end
                        end
                        
                        goal_i=goal_i+1;
                        j=j+1
                        step_id=1;
                        update_cycle=30;
                    else  % +x방향 회전
                        global imusub;
                        imu = receive(imusub);
                        quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
                        eul=quat2eul(quat);
                        
                        while 1      %@ 돌때까지 Rot를 보고 판단하며 기다린다.
                            global setpub; global setmsg;
                            setmsg_orient(quat_goal(:,goal_i));
                            send(setpub,setmsg);
                            global imusub;
                            imu = receive(imusub);
                            quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
                            eul=quat2eul(quat);
                            fprintf("waiting rotate to 0.. %f \n", eul(1))
                            if abs(eul(1)-0*pi/180)<0.1
                                fprintf("finish turn \n")
                                break;
                            end
                        end
                        goal_i=goal_i+1;
                        j=j+1
                        step_id=1;
                        update_cycle=9;
                    end
                otherwise
            end
            if goal_i==size(way1_goal,2) 
                time = rate.TotalElapsedTime;
                X=['Iteration No.: ',num2str(i), ', Arrival time: ', num2str(time),'s'];
                disp(X);
                break;
            end
        end
        if (norm(trans(1:2)-way2_goal(:,goal2_i))<1 & global_path_flag==2)  
            step_id=1;
            update_cycle=20;
            if goal2_i==size(way2_goal,2) 
                time = rate.TotalElapsedTime;
                X=['Iteration No.: ',num2str(i), ', Arrival time: ', num2str(time),'s'];
                disp(X);
                break;
            else
                while 1      %@ 돌때까지 Rot를 보고 판단하며 기다린다.
                    global setpub; global setmsg;
                    setmsg_orient(quat_goal2');
                    send(setpub,setmsg);
                    global imusub;
                    imu = receive(imusub);
                    quat=[imu.Orientation.W,imu.Orientation.X,imu.Orientation.Y,imu.Orientation.Z]; 
                    eul=quat2eul(quat);
                    fprintf("waiting rotate to quat_goal2.. %f \n", eul(1))
                    if abs(eul(1)-ret2)<0.1
                        fprintf("finish turn \n")
                        break;
                    end
                    flag=1;
                end
                [quat_goal2,ret2]=calculate_orient(way2_goal(:,goal2_i),way2_goal(:,goal2_i+1));
                goal2_i=goal2_i+1;
            end
        end
    end
    
    i=i+1;
    waitfor(rate);
end
    
rosshutdown