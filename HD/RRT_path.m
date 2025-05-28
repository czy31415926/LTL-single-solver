function [position,track,n,done,path,pathnum] = RRT_path(X,Y,Xt,Yt,X0,Y0,bar)
%{
bar=[];
for i=1:1
    bar=[bar, struct('position',[],'r',[])];
end
X=10;Y=10;Xt=9;Yt=9;X0=1;Y0=1;bar.position=[5,5];bar.r=2;color=1;
%}
%地图大小X*Y，起始点（X0，Y0），终止点（XT，YT），允许半径round=0.5,步进长r=0.5
x=X0;y=Y0;
r=0.3;
xt=Xt;yt=Yt;
round=r;
%初始化
done=0;
n=1;
track=zeros([1,4]);%前两位为坐标,最后一位为父节点下标
track(1,1:2)=[x,y];
track(1,3)=1;
track(1,4)=((track(n,1)-xt)^2+(track(n,2)-yt)^2)^0.5;

while(1)
    ran=randi([1,10],1,1);
    if ran>1
        p=[xt,yt];
    else
        p=randi([1,X],[1,2]);
    end
    distance=zeros([size(track,1),1]);
    for i=1:size(track,1)
        distance(i)=(track(i,1)-p(1))^2+(track(i,2)-p(2))^2;
    end
    [d,connect]=min(distance);%connect为随机点的连接点的track下标
    last_point=track(connect,1:2);
    cost=(p(1)-last_point(1))/((p(1)-last_point(1))^2+(p(2)-last_point(2))^2)^0.5;
    sint=(p(2)-last_point(2))/((p(1)-last_point(1))^2+(p(2)-last_point(2))^2)^0.5;
    for i=1:floor(((p(1)-last_point(1))^2+(p(2)-last_point(2))^2)^0.5/r)%最多延长至随机点
        new_point=[r*cost+last_point(1),r*sint+last_point(2)];
        %验证是否干涉
        inter=0;
        for k=1:size(bar,2)
            if (new_point(1)-bar(k).position(1))^2+(new_point(2)-bar(k).position(2))^2<bar(k).r^2
                inter=1;%检查是否干涉
            end
        end
        if inter==1
            break;%确认干涉则此点拓展结束
        else%否则加入节点树
            n=n+1;%记录节点数
            track(n,1:2)=new_point;
            track(n,3)=connect;
            track(n,4)=((track(n,1)-xt)^2+(track(n,2)-yt)^2)^0.5;
            connect=n;
            last_point=new_point;%迭代，下一个父节点
            if track(n,4)<round
                done=1;
            end
        end
    end
    if done==1||n>((X^2+Y^2)^0.5*5/r)^2%当节点数过多时还没搜索到，也跳出
        break;
    end
end
position=track(n,1:2);
path_finish=0;
if done==1
    m=n;
    pathnum=0;
    while(1)
        a=[track(m,1),track(track(m,3),1)];
        b=[track(m,2),track(track(m,3),2)];

        pathnum=pathnum+1;
        path(pathnum,:)=[track(m,1),track(m,2)];
        if path_finish==1
            break;
        end
        if m==1
            path_finish=1;
        end
        m=track(m,3);
    end
end




