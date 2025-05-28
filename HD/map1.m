function [T]=map1()
%地图大小
T.X=10;T.Y=10;
%定义初始状态
T.Q0 = [9,1];
%定义特殊节点位置
T.nodes=[];
T.N=5;%特殊节点数
T.M=5;%几种特殊节点
wrong=2^T.M-1;
N=T.N;
for i=1:N
    T.nodes=[T.nodes, struct('position',[],'atomicProp',[],'data',[])];
end
subplot(1,1,1);
T.nodes(1).atomicProp='a';T.nodes(1).position=[9,1];T.nodes(1).data=1;T.nodes(1).r=1.2;
T.nodes(2).atomicProp='b';T.nodes(2).position=[1.2,1];T.nodes(2).data=2;T.nodes(2).r=1.2;
T.nodes(3).atomicProp='c';T.nodes(3).position=[9,9];T.nodes(3).data=4;T.nodes(3).r=1.2;
T.nodes(4).atomicProp='d';T.nodes(4).position=[5,5];T.nodes(4).data=8;T.nodes(4).r=1.2;
T.nodes(5).atomicProp='e';T.nodes(5).position=[1,5];T.nodes(5).data=16;T.nodes(5).r=1.2;

% fill([8.5 9.5 9.5 8.5],[0.5 0.5 1.5 1.5],[0.8,0.8,0.8]);hold on;
% fill([0.5 1.5 1.5 0.5],[0.5 0.5 1.5 1.5],[0.8,0,0.8]);hold on;
% fill([8.5 9.5 9.5 8.5],[8.5 8.5 9.5 9.5],'b');hold on;
% fill([4.5 5.5 5.5 4.5],[4.5 4.5 5.5 5.5],'r');hold on;
% fill([0.5 1.5 1.5 0.5],[4.5 4.5 5.5 5.5],'y');hold on;
% fill([0.5 5 5 0.5],[8.5 8.5 9.5 9.5],'g');hold on;
rectangle('Position',[0.7,0.5,1,1],'Curvature',0.5,'FaceColor',[1 1 1],'EdgeColor',[0.5,0,1],'LineWidth',2);hold on;
rectangle('Position',[8.5,0.5,1,1],'Curvature',0.5,'FaceColor',[1 1 1],'EdgeColor',[0.5,0.5,0.5],'LineWidth',2);hold on;
rectangle('Position',[4.5,4.5,1,1],'Curvature',0.5,'FaceColor',[1 1 1],'EdgeColor',[1,0,0],'LineWidth',2);hold on;
rectangle('Position',[0.5,4.5,1,1],'Curvature',0.5,'FaceColor',[1 1 1],'EdgeColor',[1,1,0],'LineWidth',2);hold on;
rectangle('Position',[8.5,8.5,1,1],'Curvature',0.5,'FaceColor',[1 1 1],'EdgeColor',[1,0.5,0],'LineWidth',2);hold on;
rectangle('Position',[0.5,8.5,4.5,1],'Curvature',0.5,'FaceColor',[0 0 0],'EdgeColor',[0,0,0],'LineWidth',2);hold on;


fill([-0.5 10.5 10.5 -0.5],[-0.5 -0.5 0 0],'k');hold on;
fill([-0.5 10.5 10.5 -0.5],[10 10 10.5 10.5],'k');hold on;
fill([-0.5 0 0 -0.5],[0 0 10 10],'k');hold on;
fill([10 10.5 10.5 10],[0 0 10 10],'k');hold on;
d=1.7;
fill([2 2.4 2.4 2],[0 0 d d],'k');hold on;
fill([10-d 10 10 10-d],[2 2 2.4 2.4],'k');hold on;
fill([8 7.6 7.6 8],[10-d 10-d 10 10],'k');hold on;
for i=1:length(T.nodes)
    x=T.nodes(i).position(1)-0.2;
    y=T.nodes(i).position(2)-0.0;
    s=['ap_',num2str(i)];
    text(x,y,s);hold on;
end
    
%text(9,1,'ap_1');text(1,1,'ap_2');text(9,9,'ap_3');text(5,5,'ap_4');text(1,5,'ap_5');hold on;



%定义节点转换
T.adj = zeros([N,N]);
for i=1:N
    for j=1:N
        T.adj(i,j)=((T.nodes(i).position(1)-T.nodes(j).position(1))^2+(T.nodes(i).position(2)-T.nodes(j).position(2))^2)^0.5;
    end
end

%定义障碍
Ob=9;%障碍个数
T.bar=[];
for i=1:Ob
    T.bar=[T.bar, struct('position',[],'r',[])];
end
p=[[2.2,0.3];[2.2,0.9];[2.2,1.5];[9.7,2.2];[9.1,2.2];[8.5,2.2];[7.8,9.7];[7.8,9.1];[7.8,8.5]];
r=ones(1,9)*0.4;

for i=1:Ob
    T.bar(i).position=p(i,:);
    T.bar(i).r=r(i);
end
% for i=1:length(T.bar)
%     scatter(T.bar(i).position(1),T.bar(i).position(2),r(i)*1500,'k','filled');
%     text(T.bar(i).position(1),T.bar(i).position(2),num2str(i),'color','w');
% end


axis([-0.5 10.5 -0.5 10.5]);
set(gcf,'position',[500,70,500,500]);
set( gca, 'XTick', [], 'YTick', [] );