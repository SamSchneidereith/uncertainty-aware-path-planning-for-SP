clear all;
j =0;

for bC = 10:1:200 % Ball Constant Value
    j = j+1;
    i = 0;
    BCs(j) = bC;
for n = 1:1:10000 % Number of Nodes in the Graph
    i = i+1;
Y2(i,j) = bC*((log(1+n)/(n))^(1/2));
Y6(i,j) = bC*((log(1+n)/(n))^(1/6));
 Ns(i) = n;
end
end
%% Find where the hyper ball radius passes the delta value
test = Y2-10;
[mins,ind] = min(abs(test));
%
for i = 1:size(BCs,2)
    minsY2(i,1) = Y2(ind(i),i);
    minsY2(i,2) = Ns(ind(i));
end
% Find where the hyper ball radius passes the delta value
test2 = Y6-10;
[min2,ind2] = min(abs(test2));
%
for i = 1:size(BCs,2)
    minsY6(i,1) = Y6(ind2(i),i);
    minsY6(i,2) = Ns(ind2(i));
end
%%
figure
plot(BCs,minsY2(:,2),'DisplayName','2D Toy Problem','LineWidth',2)
grid on
xlabel('\gamma_{RRT}')
ylabel('Number of Nodes')
title('Number of Nodes where \gamma_{RRT} = \Delta')
figure
plot(BCs(1:20),minsY6(1:20,2),'DisplayName','SP Problem','LineWidth',2,'color','red')
grid on
xlabel('\gamma_{RRT}')
ylabel('Number of Nodes')
title('Number of Nodes where \gamma_{RRT} = \Delta')
%%
figure 
plot(BCs(1:70),minsY6(1:70,1))
xlabel('Ball Constant')
ylabel('Minimum Ball Radius')

figure 
plot(BCs(1:70),ind2(1:70))