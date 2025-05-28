function [position,track,done] = RRT_RHC(X,Y,Xt,Yt,X0,Y0,bar)
%{
bar=[];
for i=1:1
    bar=[bar, struct('position',[],'r',[])];
end
X=10;Y=10;Xt=9;Yt=9;X0=1;Y0=1;bar.position=[5,5];bar.r=4;
%}
%地图大小X*Y，起始点（X0，Y0），终止点（XT，YT），允许半径round=0.5，步进长r=0.5，视野半径viewR=2
r=0.5;
round=r;
viewR=2;
done=0;
T=500;
track=zeros([T,2]);
track(1,:)=[X0,Y0];

%其他参量
bound=-pi:0.1:pi;
for t=1:T
    %选取暂时终点（xt，yt）
    feasible=zeros([length(bound),3]);%前两个为坐标，最后为能量函数
    for i=1:length(bound)
        inter=0;
        a=[track(t,1)+viewR*cos(bound(i)),track(t,2)+viewR*sin(bound(i))];
        for k=1:size(bar,2)
            if (a(1)-bar(k).position(1))^2+(a(2)-bar(k).position(2))^2<bar(k).r^2
                inter=1;%检查是否干涉
                break;
            end
        end
        if inter==1
            a=[a,(X^2+Y^2)^0.5];%干涉赋值最大距离
        else
            a=[a,((a(1)-Xt)^2+(a(2)-Yt)^2)^0.5];
        end
        feasible(i,:)=a;
    end
    [en,i]=min(feasible(:,3));%挑选能量函数最小的点作为近期目标
    xt=track(t,1)+viewR*cos(bound(i));
    yt=track(t,2)+viewR*sin(bound(i));
    [position,tracknow,n,done,path,pathnum] = RRT_path(X,Y,xt,yt,track(t,1),track(t,2),bar);
    
    for i=1:pathnum
        pathnew(i,:)=path(pathnum+1-i,:);
    end
    if done==1
        track(t+1,1)=pathnew(4,1);
        track(t+1,2)=pathnew(4,2);
    else
        break;
    end
    if (track(t+1,1)-Xt)^2+(track(t+1,2)-Yt)^2<round^2
        finish=1;
        break;
    end
end
done=1;
track (all(track == 0,2),:) = [];%删除全0行
plot(track(:,1),track(:,2));
axis([0 X 0 Y]);
    
    
        
    
    





