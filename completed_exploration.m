clc;
clear;
close all;
nRbt = 3;
nIter = 60;
bound = 15;
d = 1; %step

nPop = 100;
dim = 2;
varSize = [1 dim];


ray_lngth = 1.5; % max_range value in the paper

%value delta x, delta y 
y_up = 1;
y_down = -1;
y_stay = 0;
x_right = 1;
x_left = -1;
x_stay = 0;


map = robotics.OccupancyGrid(bound,bound,15); %resolution is the number of pixels in one cell; no needs to consider it below
show (map);
hold on
% view(3)


%% obstacles
cntx = bound/2;
cnty = bound/2;
s = cnty/0.1;
A(1,s) = cntx-1;
A(1:s) = cntx;
B = cnty:0.1:bound-0.1;
C = [A; B];
C = transpose(C);
A = A/2.5;
D = [B; A];
D = transpose(D);
% q = (bound/0.1) - 10;
q = (bound/0.1);
E(1,q) = bound-1;
E(1:q) = 0.1;
I(1,q) = bound-1;
I(1:q) = bound-0.1;
% F = 1:0.1:bound-0.1;
F = 0:0.1:bound-0.1;
H = [E;F];
H = transpose(H);
G = [I;F];
G = transpose(G);
J = [F;I];
J = transpose(J);
W = [F;E];
W = transpose(W);
d_ = 0;


% upper vertical
% setOccupancy(map, C, 1)

% H left vertical
setOccupancy(map, H, 1)
% G right vertical
setOccupancy(map, G, 1)
% J upper horizontal
setOccupancy(map, J, 1)
setOccupancy(map, W, 1)



%% tunnel obstacle
% right horizontal
% setOccupancy(map, D, 1)
val = 0;

inflate(map,0.5)
show(map)
%% structure 

ranges = ray_lngth*ones(100,1); % 1.5 cells of range sensor beam
angles = linspace(3.14, -3.14, 100);
maxrange = 5;
empty_wolf.Position = [];
wolfs = repmat(empty_wolf, nPop, 1);


robot_empty.Itertn.Pose = [];
robot_empty.Itertn.V1.V1xy = [];
robot_empty.Itertn.V2.V2xy = [];
robot_empty.Itertn.V3.V3xy = [];
robot_empty.Itertn.V4.V4xy = [];
robot_empty.Itertn.V5.V5xy = [];
robot_empty.Itertn.V6.V6xy = [];
robot_empty.Itertn.V7.V7xy = [];
robot_empty.Itertn.V8.V8xy = [];
robot_empty.Itertn.V9.V9xy = [];
robot_empty.Itertn.V1.V1Index = [];
robot_empty.Itertn.V2.V2Index = [];
robot_empty.Itertn.V3.V3Index = [];
robot_empty.Itertn.V4.V4Index = [];
robot_empty.Itertn.V5.V5Index = [];
robot_empty.Itertn.V6.V6Index = [];
robot_empty.Itertn.V7.V7Index = [];
robot_empty.Itertn.V8.V8Index = [];
robot_empty.Itertn.V9.V9Index = [];
robot_empty.Itertn.RandSelect.Index = [];
robot_empty.Itertn.RandSelect.Value = [];
robot_empty.Itertn.RandSelect.Uxy = [];
robots = repmat(robot_empty, nRbt, 1);

alpha1.R1Cost = inf;
beta1.R1Cost = inf;
gamma1.R1Cost = inf;
alpha2.R2Cost = inf;
beta2.R2Cost = inf;
gamma2.R2Cost = inf;
alpha3.R3Cost = inf;
beta3.R3Cost = inf;
gamma3.R3Cost = inf;
map_before = 0;
for m=1:bound
    for n=1:bound
        vlb = checkOccupancy(map,[m,n]);
        if vlb ~= 1
            map_before = map_before + getOccupancy(map,[m,n]);
        end
    end
end


%% initialization in Itertn 1
for i=1
    for w=1:nPop
        wolfs(w).Position = unifrnd(1, bound-1, varSize);
        wolfs(w).Best.Position = wolfs(w).Position; 
        wolfs(w).State = 0;
    end
    
    for j=1:nRbt
        if j==1
            robots(j).Itertn(i).Pose = [5,5,0];
%             robots(j).Itertn(i).Pose = [6,5,0];
        end
        if j==2
            robots(j).Itertn(i).Pose = [7,9,0];
%             robots(j).Itertn(i).Pose = [7,4,0];
        end
        if j==3
            robots(j).Itertn(i).Pose = [9,4,0];
%             robots(j).Itertn(i).Pose = [8,4,0];
        end
        
        robots(j).Itertn(i).RandSelect.Uxy = [0,0];
        robots(j).Itertn(i).V1.V1Index = 1;
        robots(j).Itertn(i).V2.V2Index = 2;
        robots(j).Itertn(i).V3.V3Index = 3;
        robots(j).Itertn(i).V4.V4Index = 4;
        robots(j).Itertn(i).V5.V5Index = 5;
        robots(j).Itertn(i).V6.V6Index = 6;
        robots(j).Itertn(i).V7.V7Index = 7;
        robots(j).Itertn(i).V8.V8Index = 8;
        robots(j).Itertn(i).V9.V9Index = 9;
        
        x = robots(j).Itertn(i).Pose(1,1);
        y = robots(j).Itertn(i).Pose(1,2);
        thi = robots(j).Itertn(i).Pose(1,3);    
        
        
        hold on
        grid on
        color = ['r','b','g'];
        if j==1
            plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(1),'marker','o');
            hold on
        elseif j==2
            plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(2),'marker','o');
            hold on
        elseif j==3
            plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(3),'marker','o');
            hold on
        end
        show(map)
        hold on
        

        grid on
        for w = 1:nPop
            plot(wolfs(w).Position(1,1),wolfs(w).Position(1,2),'marker','x','MarkerSize',15,'MarkerEdgeColor','blue','LineWidth',3);
            hold on
   
   
        end
        drawnow
        show(map)
        hold on  

        

        
        
        
    end
    
end



       
a = 2;
adamp = a/nIter;
alphaCostsR1 = zeros(nIter,1);
alphaCostsR2 = zeros(nIter,1);
alphaCostsR3 = zeros(nIter,1);

map_result_arr = [];
% 
%% Main source
% 
%Cost
hold on
for i=1:nIter
    alpha1.R1Cost = inf;
    beta1.R1Cost = inf;
    gamma1.R1Cost = inf;
    alpha2.R2Cost = inf;
    beta2.R2Cost = inf;
    gamma2.R2Cost = inf;
    alpha3.R3Cost = inf;
    beta3.R3Cost = inf;
    gamma3.R3Cost = inf;
    for w = 1:nPop
        wolfs(w).State = 0;
    end
%     surf_map = size(bound-1,bound-1);
%     for m=1:bound-1
%         for n=1:bound-1
%             surf_map(m,n) = getOccupancy(map,[m,n]);
%         end
%     end
    
    if a<= 1
        alpha1.R1Cost = 0;
        beta1.R1Cost = 0;
        gamma1.R1Cost = 0;
        alpha2.R2Cost = 0;
        beta2.R2Cost = 0;
        gamma2.R2Cost = 0;
        alpha3.R3Cost = 0;
        beta3.R3Cost = 0;
        gamma3.R3Cost = 0;
        
    end
%     val = 1;
%     for w=1:nPop
%         wolfs(w).Position = unifrnd(1, bound-1, varSize);
%         wolfs(w).Best.Position = wolfs(w).Position; 
%     end
     A1 = 2*a*rand(varSize) - a;
     C1 = 2*rand(varSize);
     A2 = 2*a*rand(varSize) - a;
     C2 = 2*rand(varSize);
     A3 = 2*a*rand(varSize) - a;
     C3 = 2*rand(varSize);




%      show(map)
     hold on
     grid on
     
     for w = 1:nPop
%           hold on
          plot(wolfs(w).Position(1,1),wolfs(w).Position(1,2),'marker','x','MarkerSize',15,'MarkerEdgeColor','blue','LineWidth',3);
%           hold on
     end
     hold on
%      drawnow
     
%             show(map)
      

     hold on
    for j=1:nRbt 
        
            
        

            x = robots(j).Itertn(i).Pose(1,1);
            y = robots(j).Itertn(i).Pose(1,2);
            
            

            insertRay(map,[x,y,thi],ranges,angles,maxrange);
%             pause(1)
            [V1,V2,V3,V4,V5,V6,V7,V8,V9] = gridsOrientation(x,y);
            robots(j).Itertn(i).V1.V1Index = 1;
            robots(j).Itertn(i).V2.V2Index = 2;
            robots(j).Itertn(i).V3.V3Index = 3;
            robots(j).Itertn(i).V4.V4Index = 4;
            robots(j).Itertn(i).V5.V5Index = 5;
            robots(j).Itertn(i).V6.V6Index = 6;
            robots(j).Itertn(i).V7.V7Index = 7;
            robots(j).Itertn(i).V8.V8Index = 8;
            robots(j).Itertn(i).V9.V9Index = 9;


            robots(j).Itertn(i).V1.V1xy = V1;
            robots(j).Itertn(i).V2.V2xy = V2;
            robots(j).Itertn(i).V3.V3xy = V3;
            robots(j).Itertn(i).V4.V4xy = V4;
            robots(j).Itertn(i).V5.V5xy = V5;
            robots(j).Itertn(i).V6.V6xy = V6;
            robots(j).Itertn(i).V7.V7xy = V7;
            robots(j).Itertn(i).V8.V8xy = V8;
            robots(j).Itertn(i).V9.V9xy = V9; 
            del_v = [];
            if checkOccupancy(map,V1)== 1
                del_v = 1;
            end
            if checkOccupancy(map,V2)== 1
                del_v = [del_v,2];
            end
            if checkOccupancy(map,V3)== 1
                del_v = [del_v,3];
            end
            if checkOccupancy(map,V4)== 1
                del_v = [del_v,4];
            end
            if checkOccupancy(map,V5)== 1
                del_v = [del_v,5];
            end
            if checkOccupancy(map,V6)== 1
                del_v = [del_v,6];
            end
            if checkOccupancy(map,V7)== 1
                del_v = [del_v,7];
            end
            if checkOccupancy(map,V8)== 1
                del_v = [del_v,8];
            end

            
                for w=1:nPop

                    if j==1
                        fc = [wolfs(w).Position(1,1),wolfs(w).Position(1,2),x,y];
                        Hh = DistanceCost(fc);
                        
                        w_occ = getOccupancy(map, [wolfs(w).Position(1,1), wolfs(w).Position(1,2)]); 
                        if a>=1
                            wolfs(w).R1Cost = Hh;
                            if w_occ < 0.4
                                wolfs(w).Best.R1Cost = Hh+150;
                            else
                                wolfs(w).Best.R1Cost = Hh;
                            end
                        end
                        if a<=1
                            wolfs(w).R1Cost = w_occ/Hh;
%                             if w_occ == 0.0010
%                                 wolfs(w).Best.R1Cost = w_occ;
%                             elseif w_occ < 0.4
%                                wolfs(w).Best.R1Cost = w_occ; 
%                             end
%                                 wolfs(w).Best.R1Cost = w_occ;
                            wolfs(w).Best.R1Cost = w_occ/Hh;
                        end

                    end
                    if j==2
                        fc = [wolfs(w).Position(1,1),wolfs(w).Position(1,2),x,y];
                        Hh = DistanceCost(fc);
                        
        %                 wolfs(w).Best.R2Cost = Hh;
                        w_occ = getOccupancy(map, [wolfs(w).Position(1,1), wolfs(w).Position(1,2)]); 
                       if a>=1
                           wolfs(w).R2Cost = Hh;
                            if w_occ < 0.4
                                wolfs(w).Best.R2Cost = Hh+150;
                            else
                                wolfs(w).Best.R2Cost = Hh;
                            end
                        end
                        if a<=1
                            wolfs(w).R2Cost = w_occ/Hh;
%                             if w_occ == 0.0010
%                                wolfs(w).Best.R2Cost = w_occ; 
%                             elseif w_occ < 0.4
%                                wolfs(w).Best.R2Cost = Hh*w_occ; 
%                             end
%                                 wolfs(w).Best.R2Cost = w_occ;
                            wolfs(w).Best.R2Cost = w_occ/Hh;
                        end
                    end
                    if j==3
                        fc = [wolfs(w).Position(1,1),wolfs(w).Position(1,2),x,y];
                        Hh = DistanceCost(fc);
                        w_occ = getOccupancy(map, [wolfs(w).Position(1,1), wolfs(w).Position(1,2)]); 
                        
                        if a>=1
                            wolfs(w).R3Cost = Hh;
                            if w_occ < 0.4
                                wolfs(w).Best.R3Cost = Hh+150;
                            else
                                wolfs(w).Best.R3Cost = Hh;
                            end
                        end
                        if a<=1
                            wolfs(w).R3Cost = w_occ/Hh;
%                             if w_occ == 0.0010
%                                 wolfs(w).Best.R3Cost = w_occ;
%                             elseif w_occ < 0.4
%                                wolfs(w).Best.R3Cost = Hh*w_occ; 
%                             end
%                                 wolfs(w).Best.R3Cost = w_occ;
                              wolfs(w).Best.R3Cost = w_occ/Hh;
                        end
                    end
                end
                
                
                
                if a>=1
                

                     if j == 1
                        for w=1:nPop
                            if wolfs(w).Best.R1Cost < alpha1.R1Cost
                                gamma1 = beta1;
                                beta1 = alpha1;
                                alpha1.R1Cost = wolfs(w).R1Cost;
                                alpha1.Position = wolfs(w).Best.Position;
                                a_state = w;

                            elseif wolfs(w).Best.R1Cost < beta1.R1Cost
                                gamma1 = beta1;
                                beta1.R1Cost = wolfs(w).R1Cost;
                                beta1.Position = wolfs(w).Best.Position;
                                b_state = w;


                            elseif wolfs(w).Best.R1Cost < gamma1.R1Cost
                                gamma1.R1Cost = wolfs(w).R1Cost;
                                gamma1.Position = wolfs(w).Best.Position;
                                g_state = w;
                            end    
                        end
                        wolfs(a_state).State = 1;
                        wolfs(b_state).State = 1;
                        wolfs(g_state).State = 1;
                    end
                    if j == 2
                        for w=1:nPop
                            if wolfs(w).Best.R2Cost < alpha2.R2Cost && wolfs(w).State == 0
                                gamma2 = beta2;
                                beta2 = alpha2;
                                alpha2.R2Cost = wolfs(w).R2Cost;
                                alpha2.Position = wolfs(w).Best.Position;
                                a_state = w;

                            elseif wolfs(w).Best.R2Cost < beta2.R2Cost && wolfs(w).State == 0
                                gamma2 = beta2;
                                beta2.R2Cost = wolfs(w).R2Cost;
                                beta2.Position = wolfs(w).Best.Position;
                                b_state = w;
                            elseif wolfs(w).Best.R2Cost < gamma2.R2Cost && wolfs(w).State == 0

                                gamma2.R2Cost = wolfs(w).R2Cost;
                                gamma2.Position = wolfs(w).Best.Position;
                                g_state = w;
                            end    
                        end
                        wolfs(a_state).State = 2;
                        wolfs(b_state).State = 2;
                        wolfs(g_state).State = 2;
                    end
                    if j == 3
                        for w=1:nPop
                            if wolfs(w).Best.R3Cost < alpha3.R3Cost && wolfs(w).State == 0
                                gamma3 = beta3;
                                beta3 = alpha3;
                                alpha3.R3Cost = wolfs(w).R3Cost;
                                alpha3.Position = wolfs(w).Best.Position;
                                a_state = w;
                            elseif wolfs(w).Best.R3Cost < beta3.R3Cost && wolfs(w).State == 0
                                gamma3 = beta3;
                                beta3.R3Cost = wolfs(w).R3Cost;
                                beta3.Position = wolfs(w).Best.Position;
                                b_state = w;

                            elseif wolfs(w).Best.R3Cost < gamma3.R3Cost && wolfs(w).State == 0
                                gamma3.R3Cost = wolfs(w).R3Cost;
                                gamma3.Position = wolfs(w).Best.Position;
                                g_state = w;
                            end    
                        end
                        wolfs(a_state).State = 3;
                        wolfs(b_state).State = 3;
                        wolfs(g_state).State = 3;
                    end
                end
                
                
                if a<=1

                     if j == 1
                        for w=1:nPop
                            if wolfs(w).Best.R1Cost > alpha1.R1Cost
                                gamma1 = beta1;
                                beta1 = alpha1;
                                alpha1.R1Cost = wolfs(w).R1Cost;
                                alpha1.Position = wolfs(w).Best.Position;
                                a_state = w;

                            elseif wolfs(w).Best.R1Cost > beta1.R1Cost
                                gamma1 = beta1;
                                beta1.R1Cost = wolfs(w).R1Cost;
                                beta1.Position = wolfs(w).Best.Position;
                                b_state = w;


                            elseif wolfs(w).Best.R1Cost > gamma1.R1Cost
                                gamma1.R1Cost = wolfs(w).R1Cost;
                                gamma1.Position = wolfs(w).Best.Position;
                                g_state = w;
                            end    
                        end
                        wolfs(a_state).State = 1;
                        wolfs(b_state).State = 1;
                        wolfs(g_state).State = 1;
                    end
                    if j == 2
                        for w=1:nPop
                            if wolfs(w).Best.R2Cost > alpha2.R2Cost && wolfs(w).State == 0
                                gamma2 = beta2;
                                beta2 = alpha2;
                                alpha2.R2Cost = wolfs(w).R2Cost;
                                alpha2.Position = wolfs(w).Best.Position;
                                a_state = w;

                            elseif wolfs(w).Best.R2Cost > beta2.R2Cost && wolfs(w).State == 0
                                gamma2 = beta2;
                                beta2.R2Cost = wolfs(w).R2Cost;
                                beta2.Position = wolfs(w).Best.Position;
                                b_state = w;
                            elseif wolfs(w).Best.R2Cost > gamma2.R2Cost && wolfs(w).State == 0

                                gamma2.R2Cost = wolfs(w).R2Cost;
                                gamma2.Position = wolfs(w).Best.Position;
                                g_state = w;
                            end    
                        end
                        wolfs(a_state).State = 2;
                        wolfs(b_state).State = 2;
                        wolfs(g_state).State = 2;
                    end
                    if j == 3
                        for w=1:nPop
                            if wolfs(w).Best.R3Cost > alpha3.R3Cost && wolfs(w).State == 0
                                gamma3 = beta3;
                                beta3 = alpha3;
                                alpha3.R3Cost = wolfs(w).R3Cost;
                                alpha3.Position = wolfs(w).Best.Position;
                                a_state = w;
                            elseif wolfs(w).Best.R3Cost > beta3.R3Cost && wolfs(w).State == 0
                                gamma3 = beta3;
                                beta3.R3Cost = wolfs(w).R3Cost;
                                beta3.Position = wolfs(w).Best.Position;
                                b_state = w;

                            elseif wolfs(w).Best.R3Cost > gamma3.R3Cost && wolfs(w).State == 0
                                gamma3.R3Cost = wolfs(w).R3Cost;
                                gamma3.Position = wolfs(w).Best.Position;
                                g_state = w;
                            end    
                        end
                        wolfs(a_state).State = 3;
                        wolfs(b_state).State = 3;
                        wolfs(g_state).State = 3;
                    end
                    
                end
                
                if a>=1
                
                    if j == 1
                            Dalpha1 = abs(C1.*alpha1.Position - [x,y]);
                            X1 = alpha1.Position - A1.*Dalpha1;
                            Dbeta1 = abs(C2.*beta1.Position - [x,y]);
                            X2 = beta1.Position - A2.*Dbeta1;
                            Dgamma1 = abs(C3.*gamma1.Position - [x,y]);
                            X3 = gamma1.Position - A3.*Dgamma1;

                            abg_r1 = (X1 + X2 + X3)/3;


                                min_abg_r1_V1 = pdist([abg_r1;V1],'euclidean');
                                min_abg_r1_V2 = pdist([abg_r1;V2],'euclidean');
                                min_abg_r1_V3 = pdist([abg_r1;V3],'euclidean');                       
                                min_abg_r1_V4 = pdist([abg_r1;V4],'euclidean');                       
                                min_abg_r1_V5 = pdist([abg_r1;V5],'euclidean');                       
                                min_abg_r1_V6 = pdist([abg_r1;V6],'euclidean');                        
                                min_abg_r1_V7 = pdist([abg_r1;V7],'euclidean');                        
                                min_abg_r1_V8 = pdist([abg_r1;V8],'euclidean');

                                mabgr1 = [min_abg_r1_V1,min_abg_r1_V2,min_abg_r1_V3,min_abg_r1_V4,min_abg_r1_V5,...
                                min_abg_r1_V6,min_abg_r1_V7,min_abg_r1_V8];
                                if ~isempty(del_v)
                                    mabgr1(del_v(1,:))=inf;
                                end

                            [wlf_r1_value, wlf_r1_index ] = min(mabgr1);
                            robots(j).Itertn(i).RandSelect.Index = wlf_r1_index;
                            robots(j).Itertn(i).RandSelect.Value = wlf_r1_value;
                            alphaCostsR1(i) = alpha1.R1Cost;
                     end
                     if j == 2
                            Dalpha2 = abs(C1.*alpha2.Position - [x,y]);
                            X1 = alpha2.Position - A1.*Dalpha2;
                            Dbeta2 = abs(C2.*beta2.Position - [x,y]);
                            X2 = beta2.Position - A2.*Dbeta2;
                            Dgamma2 = abs(C3.*gamma2.Position - [x,y]);
                            X3 = gamma2.Position - A3.*Dgamma2;

                            abg_r2 = (X1 + X2 + X3)/3;
                                min_abg_r2_V1 = pdist([abg_r2;V1],'euclidean');
                                min_abg_r2_V2 = pdist([abg_r2;V2],'euclidean');
                                min_abg_r2_V3 = pdist([abg_r2;V3],'euclidean');                       
                                min_abg_r2_V4 = pdist([abg_r2;V4],'euclidean');                       
                                min_abg_r2_V5 = pdist([abg_r2;V5],'euclidean');                       
                                min_abg_r2_V6 = pdist([abg_r2;V6],'euclidean');                        
                                min_abg_r2_V7 = pdist([abg_r2;V7],'euclidean');                        
                                min_abg_r2_V8 = pdist([abg_r2;V8],'euclidean');
                                mabgr2 = [min_abg_r2_V1,min_abg_r2_V2,min_abg_r2_V3,min_abg_r2_V4,min_abg_r2_V5,...
                                min_abg_r2_V6,min_abg_r2_V7,min_abg_r2_V8];
                            if ~isempty(del_v)
                                mabgr2(del_v(1,:))=inf;
                            end

                            [wlf_r2_value, wlf_r2_index ] = min(mabgr2);
                            robots(j).Itertn(i).RandSelect.Index = wlf_r2_index;
                            robots(j).Itertn(i).RandSelect.Value = wlf_r2_value;
                            alphaCostsR2(i) = alpha2.R2Cost;
                     end
                     if j == 3
                            Dalpha3 = abs(C1.*alpha3.Position - [x,y]);
                            X1 = alpha3.Position - A1.*Dalpha3;
                            Dbeta3 = abs(C2.*beta3.Position - [x,y]);
                            X2 = beta3.Position - A2.*Dbeta3;
                            Dgamma3 = abs(C3.*gamma3.Position - [x,y]);
                            X3 = gamma3.Position - A3.*Dgamma3;

                            abg_r3 = (X1 + X2 + X3)/3;
                            min_abg_r3_V1 = pdist([abg_r3;V1],'euclidean');
                            min_abg_r3_V2 = pdist([abg_r3;V2],'euclidean');
                            min_abg_r3_V3 = pdist([abg_r3;V3],'euclidean');                       
                            min_abg_r3_V4 = pdist([abg_r3;V4],'euclidean');                       
                            min_abg_r3_V5 = pdist([abg_r3;V5],'euclidean');                       
                            min_abg_r3_V6 = pdist([abg_r3;V6],'euclidean');                        
                            min_abg_r3_V7 = pdist([abg_r3;V7],'euclidean');                        
                            min_abg_r3_V8 = pdist([abg_r3;V8],'euclidean');
                            mabgr3 = [min_abg_r3_V1,min_abg_r3_V2,min_abg_r3_V3,min_abg_r3_V4,min_abg_r3_V5,...
                                min_abg_r3_V6,min_abg_r3_V7,min_abg_r3_V8];
                            if ~isempty(del_v)
                                mabgr3(del_v(1,:))=inf;
                            end

                            [wlf_r3_value, wlf_r3_index ] = min(mabgr3);
                            robots(j).Itertn(i).RandSelect.Index = wlf_r3_index;
                            robots(j).Itertn(i).RandSelect.Value = wlf_r3_value;
                            alphaCostsR3(i) = alpha3.R3Cost;
                     end  
                end
                
                if a<=1
                    if j == 1
                        wp_dist_V1 = pdist([alpha1.Position;V1],'euclidean');
                        wp_dist_V2 = pdist([alpha1.Position;V2],'euclidean');
                        wp_dist_V3 = pdist([alpha1.Position;V3],'euclidean');
                        wp_dist_V4 = pdist([alpha1.Position;V4],'euclidean');
                        wp_dist_V5 = pdist([alpha1.Position;V5],'euclidean');
                        wp_dist_V6 = pdist([alpha1.Position;V6],'euclidean');
                        wp_dist_V7 = pdist([alpha1.Position;V7],'euclidean');
                        wp_dist_V8 = pdist([alpha1.Position;V8],'euclidean');
                        wp_dist_1 = [wp_dist_V1,wp_dist_V2,wp_dist_V3,wp_dist_V4,wp_dist_V5,...
                                wp_dist_V6,wp_dist_V7,wp_dist_V8];
                        [wp_dist_value, wp_dist_index ] = min(wp_dist_1);
                        robots(j).Itertn(i).RandSelect.Index = wp_dist_index;
                        robots(j).Itertn(i).RandSelect.Value = wp_dist_value;
                        alphaCostsR1(i) = alpha1.R1Cost;
                    end
                    
                    if j == 2
                        wp_dist_V1 = pdist([alpha2.Position;V1],'euclidean');
                        wp_dist_V2 = pdist([alpha2.Position;V2],'euclidean');
                        wp_dist_V3 = pdist([alpha2.Position;V3],'euclidean');
                        wp_dist_V4 = pdist([alpha2.Position;V4],'euclidean');
                        wp_dist_V5 = pdist([alpha2.Position;V5],'euclidean');
                        wp_dist_V6 = pdist([alpha2.Position;V6],'euclidean');
                        wp_dist_V7 = pdist([alpha2.Position;V7],'euclidean');
                        wp_dist_V8 = pdist([alpha2.Position;V8],'euclidean');
                        wp_dist_2 = [wp_dist_V1,wp_dist_V2,wp_dist_V3,wp_dist_V4,wp_dist_V5,...
                                wp_dist_V6,wp_dist_V7,wp_dist_V8];
                        [wp_dist_value, wp_dist_index ] = min(wp_dist_2);
                        robots(j).Itertn(i).RandSelect.Index = wp_dist_index;
                        robots(j).Itertn(i).RandSelect.Value = wp_dist_value;
                        alphaCostsR2(i) = alpha2.R2Cost;
                    end
                    
                    if j == 3
                        wp_dist_V1 = pdist([alpha3.Position;V1],'euclidean');
                        wp_dist_V2 = pdist([alpha3.Position;V2],'euclidean');
                        wp_dist_V3 = pdist([alpha3.Position;V3],'euclidean');
                        wp_dist_V4 = pdist([alpha3.Position;V4],'euclidean');
                        wp_dist_V5 = pdist([alpha3.Position;V5],'euclidean');
                        wp_dist_V6 = pdist([alpha3.Position;V6],'euclidean');
                        wp_dist_V7 = pdist([alpha3.Position;V7],'euclidean');
                        wp_dist_V8 = pdist([alpha3.Position;V8],'euclidean');
                        wp_dist_3 = [wp_dist_V1,wp_dist_V2,wp_dist_V3,wp_dist_V4,wp_dist_V5,...
                                wp_dist_V6,wp_dist_V7,wp_dist_V8];
                        [wp_dist_value, wp_dist_index ] = min(wp_dist_3);
                        robots(j).Itertn(i).RandSelect.Index = wp_dist_index;
                        robots(j).Itertn(i).RandSelect.Value = wp_dist_value;
                        alphaCostsR3(i) = alpha3.R3Cost;
                    end
                    
                end

    %          insertRay(map,[x,y,thi],ranges,angles,maxrange);               
    
           
    end

    if i==1
       a;
    else
       a = a - adamp;
        
       disp("a = " + a);
    end
      
   %%  reduce Utilities
   hold on
    for j=1:nRbt
            
            t_ = robots(j).Itertn(i).RandSelect.Index;
            if t_ == 1
                robots(j).Itertn(i).RandSelect.Uxy = robots(j).Itertn(i).V1.V1xy;
                disp("Selected x = " + robots(j).Itertn(i).RandSelect.Uxy(1,1) + ", y = " + robots(j).Itertn(i).RandSelect.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V1.V1xy,0];
            end
        
            if t_ == 2

                robots(j).Itertn(i).RandSelect.Uxy = robots(j).Itertn(i).V2.V2xy;
                disp("Selected x = " + robots(j).Itertn(i).RandSelect.Uxy(1,1) + ", y = " + robots(j).Itertn(i).RandSelect.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V2.V2xy,0];
            end

            if t_ == 3
                robots(j).Itertn(i).RandSelect.Uxy = robots(j).Itertn(i).V3.V3xy;
                disp("Selected x = " + robots(j).Itertn(i).RandSelect.Uxy(1,1) + ", y = " + robots(j).Itertn(i).RandSelect.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V3.V3xy,0];
            end

            if t_ == 4
                robots(j).Itertn(i).RandSelect.Uxy = robots(j).Itertn(i).V4.V4xy;
                disp("Selected x = " + robots(j).Itertn(i).RandSelect.Uxy(1,1) + ", y = " + robots(j).Itertn(i).RandSelect.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V4.V4xy,0];
            end

            if t_ == 5
                robots(j).Itertn(i).RandSelect.Uxy = robots(j).Itertn(i).V5.V5xy;
                disp("Selected x = " + robots(j).Itertn(i).RandSelect.Uxy(1,1) + ", y = " + robots(j).Itertn(i).RandSelect.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V5.V5xy,0];
            end

            if t_ == 6
                robots(j).Itertn(i).RandSelect.Uxy = robots(j).Itertn(i).V6.V6xy;
                disp("Selected x = " + robots(j).Itertn(i).RandSelect.Uxy(1,1) + ", y = " + robots(j).Itertn(i).RandSelect.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V6.V6xy,0];
            end

            if t_ == 7
                robots(j).Itertn(i).RandSelect.Uxy = robots(j).Itertn(i).V7.V7xy;
                disp("Selected x = " + robots(j).Itertn(i).RandSelect.Uxy(1,1) + ", y = " + robots(j).Itertn(i).RandSelect.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V7.V7xy,0];
            end
            if t_ == 8
                robots(j).Itertn(i).RandSelect.Uxy = robots(j).Itertn(i).V8.V8xy;
                disp("Selected x = " + robots(j).Itertn(i).RandSelect.Uxy(1,1) + ", y = " + robots(j).Itertn(i).RandSelect.Uxy(1,2));
                robots(j).Itertn(i+1).Pose = [robots(j).Itertn(i).V8.V8xy,0];
            end
        
        

        hold on
        grid on
        color = ['r','b','g'];
        if j==1
            hold on
            h1=plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(1),'marker','o');
            set(h1, 'markerfacecolor', get(h1, 'color'));
            

        elseif j==2
            hold on
            h2=plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(2),'marker','o');
            set(h2, 'markerfacecolor', get(h2, 'color'));
            

        elseif j==3
            hold on
            h3=plot(robots(j).Itertn(i).Pose(1,1),robots(j).Itertn(i).Pose(1,2),'color',color(3),'marker','o');
            set(h3, 'markerfacecolor', get(h3, 'color'));
            

        end
         hold on

       

    end
    
    map_after = 0;
    for m=1:bound
        for n=1:bound
            vlb = checkOccupancy(map,[m,n]);
            if vlb ~= 1
                map_after = map_after + getOccupancy(map,[m,n]);
            end
        end
    end

    map_result_prc = (100-(map_after/map_before)*100);
    map_result_arr(i) = map_result_prc; 

    
%      pause(0.5)
    hold on
    drawnow
    show(map)
    grid on

    title(['Iteration = ', num2str(i), '; #WP = ', num2str(nPop), '; a = ', num2str(a), '; %Explored = ', num2str(map_result_prc)]);
    hold on
%     drawnow 
    
    

    
end

hold on
grid on

for w = 1:nPop
     hold on
     plot(wolfs(w).Position(1,1),wolfs(w).Position(1,2),'marker','x','MarkerSize',15,'MarkerEdgeColor','blue','LineWidth',3);
     hold on
   
   
end

figure(2)
show(map)
hold on
title(['Iteration = ', num2str(nIter), '; #Waypoints = ', num2str(nPop), '; %Explored = ', num2str(map_result_prc)]);
hold on
for j=1:nRbt
    for it_=1:nIter
    
            if j==1
                h1 = plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(it_).Pose(1,2),'color',color(1),'marker','o');
                set(h1, 'markerfacecolor', get(h1, 'color'));
                hold on
            end
            if j==2
                h2 = plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(it_).Pose(1,2),'color',color(2),'marker','s');
                set(h2, 'markerfacecolor', get(h2, 'color'));
                hold on
            end
            if j==3
                h3 = plot(robots(j).Itertn(it_).Pose(1,1),robots(j).Itertn(it_).Pose(1,2),'color',color(3),'marker','d');
                set(h3, 'markerfacecolor', get(h3, 'color'));
                hold on

            end
            if it_>1
                if j==1
                    plot([robots(j).Itertn(it_-1).Pose(1,1),robots(j).Itertn(it_).Pose(1,1)],...
                    [robots(j).Itertn(it_-1).Pose(1,2),robots(j).Itertn(it_).Pose(1,2)],'r');
                end
                if j==2
                    plot([robots(j).Itertn(it_-1).Pose(1,1),robots(j).Itertn(it_).Pose(1,1)],...
                    [robots(j).Itertn(it_-1).Pose(1,2),robots(j).Itertn(it_).Pose(1,2)],'b');
                end
                if j==3
                    plot([robots(j).Itertn(it_-1).Pose(1,1),robots(j).Itertn(it_).Pose(1,1)],...
                    [robots(j).Itertn(it_-1).Pose(1,2),robots(j).Itertn(it_).Pose(1,2)],'g');
                end
               
              
            end
     end
        
end




% show(map)
    
% map_after = 0;
% for m=1:bound
%     for n=1:bound
%         vlb = checkOccupancy(map,[m,n]);
%         if vlb ~= 1
%             map_after = map_after + getOccupancy(map,[m,n]);
%         end
%     end
% end
% 
% map_result_prc = (100-(map_after/map_before)*100);
fin_map = size(bound,bound);
for m=1:bound
    for n=1:bound
        fin_map(m,n) = getOccupancy(map,[m,n]);
    end
end

occ_waypoints = [];
for i=1:nPop
    occ_waypoints(i) = getOccupancy(map,[wolfs(i).Position(1,1),wolfs(i).Position(1,2)]);
end


figure(6);
xaxis_p = 1:1:nIter;
% yaxis_p = [map_result_arr(1),map_result_arr(nIter)];
% bar(map_result_arr);
xlabel('Iterations'); 
ylabel('Explored map in percentage ');
title('Exploration rate');
hold on
err_low(1) = 0;
err_down(nIter) = 0;
for i=1:nIter-1
    err_low(i+1) = map_result_arr(i+1)-map_result_arr(i);
end
for i=1:nIter-1
    err_down(i) = map_result_arr(i+1)-map_result_arr(i);
end
hold on
errorbar(xaxis_p,map_result_arr,err_low,err_down, 'LineWidth',1);

figure(3);
hf2.Color='w'; %Set background color of figure window
xlim([0,nIter])
set(gca,'XTick',alphaCostsR1(nIter):alphaCostsR1(1));
set(gca,'YTick',0:nIter);
semilogy(alphaCostsR1, 'LineWidth', 2);
xlabel('Iterations'); 
ylabel('Alpha cost ');


% set(gca, 'FontSize', fsz, 'LineWidth', alw); %<- Set properties

title('Alpha wolf optimization of robot 1');


figure(4);
alw = 0.95;
fsz = 17;      % Fontsize
% hf3 = figure(4); %Open figure and keep handle
% hf3=colordef(hf3,'white'); %Set color scheme
hf3.Color='w'; %Set background color of figure window
xlim([0,nIter])
set(gca,'XTick',alphaCostsR2(nIter):alphaCostsR2(1));
set(gca,'YTick',0:nIter);
semilogy(alphaCostsR2, 'LineWidth', 2);
xlabel('Iterations'); 
ylabel('Alpha cost');

% set(gca, 'FontSize', fsz, 'LineWidth', alw); %<- Set properties

title('Alpha wolf optimization of robot 2');

figure(5);
hf4 = figure(5); %Open figure and keep handle
% hf4=colordef(hf4,'white'); %Set color scheme
% hf4.Color='w'; %Set background color of figure window
xlim([0,nIter])
ylim([min(alphaCostsR3),max(alphaCostsR3)])
set(gca,'XTick',min(alphaCostsR3):max(alphaCostsR3));
set(gca,'YTick',0:nIter);
semilogy(alphaCostsR3, 'LineWidth', 2);
xlabel('Iterations'); 
ylabel('Alpha cost');

% set(gca, 'FontSize', fsz, 'LineWidth', alw); %<- Set properties
title('Alpha wolf optimization of robot 3');





