function [D,P] = makeP(B,T)
P=zeros([T.N,T.N,length(B.S),length(B.S)]);
P=P+10*(T.X^2+T.Y^2)^0.5;
for i=1:length(T.nodes)
    data(i)=T.nodes(i).data;
end
for i=1:length(B.S)
    for j=1:length(B.S)
        a=[];
        for k=1:T.M
            if ~isempty(find(cell2mat(B.trans(i,j))==2^(k-1)))
                a=[a,2^(k-1)];
            end
        end
        if ~isempty(a)
            nodes=[];
            for k=1:length(a)
                nodes=[nodes,find(data==a(k))];%寻找可行节点
            end
            for k=1:T.N
                if find(nodes==k)
                    P(:,k,i,j)=T.adj(:,k);
                else
                    c=zeros([length(a),1]);
                    for h=1:length(a)
                        different=xor(bitget(T.nodes(k).data,1:8),bitget(a(h),1:8));%记录不同的位数,互异为1
                        c(h)=length(find(different==1));
                    end
                    word_diff=min(c);
                    P(:,k,i,j)=T.adj(:,k)+(T.X^2+T.Y^2)^0.5*word_diff;
                end
            end         
        end
    end
end
for i=1:length(T.nodes)
    for j=1:length(T.nodes)
        for k=1:length(B.S)
            for h=1:length(B.S)
                D((i-1)*length(B.S)+k,(j-1)*length(B.S)+h)=P(i,j,k,h);
            end
        end
    end
end





