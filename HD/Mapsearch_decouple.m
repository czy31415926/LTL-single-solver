close all
clear all
warning off all
addpath('.\LTL_Toolbox','.\Plot');

%% 构造自动机
tic
T=map1();%构建DTS
%formula='!(p4|p5)U(p2|p3)&GFp4&!p5Up4&GFp5';
%formula='GFp1&GFp2&GFp3&GFp4&GFp5';
formula='((!p2)Up1)&Fp2&(!(p4|p5)U(p3&!(p4|p5)))&GFp4&GFp5';

N_p=T.M;%命题数
alphabet=alphabet_set(obtainAlphabet(N_p));
B=create_buchi(formula,alphabet);
toc
disp(['构造时间: ',num2str(toc)]);

%% 顶层规划部分
tic
[D,P]=makeP(B,T);%构架小型加权转换系统
%确定初始状态和最终状态
S0=1;
F=[];
for i=1:length(B.F)
    for j=1:length(T.nodes)
        F=[F;(j-1)*length(B.S)+B.F(i)];
    end
end
distance_pre=10*T.N*(T.X^2+T.Y^2)^0.5;
%前缀
for i=1:length(F)
    [distance,path]=dijkstra(D,S0,F(i));
    if distance<distance_pre&&~isempty(path)
        distance_pre=distance;
        path_pre=path;
    end
end
stateS=zeros([length(path_pre),1]);
stateT=zeros([length(path_pre),1]);
for i=1:length(path_pre)
    stateS(i)=mod(path_pre(i),length(B.S));
    if stateS(i)==0
        stateS(i)=length(B.S);
    end
    stateT(i)=ceil(path_pre(i)/length(B.S));
end
%去除停滞状态
for i=2:length(stateT)
    if stateT(i)==stateT(i-1)
        stateT(i-1)=0;
    end
end
stateT(find(stateT==0))=[];
stateT';
stateT_pre=stateT;
stateS_pre=stateS;
toc
disp(['前缀顶层规划时间: ',num2str(toc)]);

%后缀
tic
distance_suf=10*T.N*(T.X^2+T.Y^2)^0.5;
for i=1:length(D)
    %不放松
    if D(path_pre(end),i)<((T.X^2+T.Y^2)*0.9)^0.5
        [distance,path]=dijkstra(D,i,path_pre(end));
        distance=distance+D(path_pre(end),i);
        if distance<distance_suf
            distance_suf=distance;
            path_suf=path;
        end
    end
end
stateS=zeros([length(path_suf),1]);
stateT=zeros([length(path_suf),1]);
for i=1:length(path_suf)
    stateS(i)=mod(path_suf(i),length(B.S));
    if stateS(i)==0
        stateS(i)=length(B.S);
    end
    stateT(i)=ceil(path_suf(i)/length(B.S));
end
%去除停滞状态
for i=2:length(stateT)
    if stateT(i)==stateT(i-1)
        stateT(i-1)=0;
    end
end
stateT(find(stateT==0))=[];
stateT';
stateT_suf=stateT;
stateS_suf=stateS;
%% get BAR
%bar=[struct('ap',[])];
bar=[];
for i=1:length(stateT_pre)
    setap=[];
    for j=1:length(T.nodes)
        if j==stateT_pre(i)
            continue
        end
        plusdata=T.nodes(j).data+T.nodes(stateT_pre(i)).data;
        if isempty(find(cell2mat(B.trans(stateS_pre(i),stateS_pre(i+1)))==plusdata))
            setap=[setap,j];
        end
    end
    bar=[bar,struct('ap',setap)];
end




toc
disp(['后缀顶层规划时间: ',num2str(toc)]);


%% 底层路径规划
tic
%绘制路径
xt(1)=T.Q0(1);yt(1)=T.Q0(2);
x0(1)=xt(1);y0(1)=yt(1);
Tbar2=T.bar;
Tbar2=[Tbar2,struct('position',[5,5.25],'r',[1.2])];
PATH=[];
for i=2:length(stateT_pre)
    p=T.nodes(stateT_pre(i)).position;
    xt(i)=p(1);yt(i)=p(2);
    %[position,track,nodenum,done,path,pathnumber]=RRT_path(T.X,T.Y,xt(i),yt(i),x0(i-1),y0(i-1),T.bar);
    %x0(i)=path(1,1);y0(i)=path(1,2);
    Tbar=T.bar;%Tbar初始化
    for j=1:length(bar(i).ap)
        Tbar=[Tbar,struct('position',T.nodes(bar(i).ap(j)).position,'r',T.nodes(bar(i).ap(j)).r)];
    end
    [position,path,done] = RRT_RHC(T.X,T.Y,xt(i),yt(i),x0(i-1),y0(i-1),Tbar);
    PATH=[PATH;path];
    x0(i)=path(end,1);y0(i)=path(end,2);
    %打印轨迹
    draw_finish=0;
    if done==1
        plot(path(:,1),path(:,2),'color',[abs(cos(2*pi*i/6)),abs(cos(2*pi*i/3+2/3*pi)),abs(cos(2*pi*i/2+4/3*pi))]);
        hold on
        scatter(path(:,1),path(:,2),10,'k','filled');
        hold on
        axis([-0.5 10.5 -0.5 10.5]);
    end
end
toc
disp(['前缀底层规划时间: ',num2str(toc)]);




