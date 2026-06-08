%% Color bar test 2:
% put everything in a struct
% sort the table based on color value
% put each color bin in it's own matrix
% plot the matrices one at a time


%% Showing the amount of noise addition based on position:
R = 5*[0.0844 -2.8739e-6; -2.8739e-6 0.0843]; % Observation Noise
[X,Y] = meshgrid(-145:1:145,490:1:790);
Xrange = 145-(-145);
Xmin = -145;
Yrange = 790-490;
Ymin = 490;
% Calculate the corresponding Z Value:
n = 1000;
Zall = NaN(size(X,1),size(X,2),n);
for i = 1:n
    for xind = 1:size(X,1)
        for yind = 1:size(X,2)
            if PosCheck2D([X(xind,yind),Y(xind,yind)]')
                continue
            else
                w(1) = (X(xind,yind)-Xmin)/Xrange;
                w(2) = (Y(xind,yind)-Ymin)/Yrange;
                v = w.*normrnd(0,diag(R));
                Zall(xind,yind,i) = norm(v);
            end
        end
    end
    if rem(i,10) == 0
        fprintf('loop %i\n',i);
    end
end
Z = mean(Zall,3);

%%
NoiseSurf = surf(X,Y,Z);
NoiseSurf.EdgeAlpha = 0;
view([0,90])
colormap('jet')
c = colorbar;
c.Label.String = 'Noise Addition';
title('Average Noise Addition based on Position');
xlabel('X Position');
ylabel('Y Position');
%%
NoiseSurf = surf(X,Y,Z);
NoiseSurf.EdgeAlpha = 0.1;
view([0,0])
colormap('jet')
c = colorbar;
c.Label.String = 'Noise Addition';
title('Average Noise Addition based on Position');
xlabel('X Position');
ylabel('Y Position');