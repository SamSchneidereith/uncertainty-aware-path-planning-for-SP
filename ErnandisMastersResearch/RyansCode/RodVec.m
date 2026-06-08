function [ out ] = RodVec( mat )
%RodVec takes a matrix input and outputs the vector 
%   mat: 3x3 rotation matrix
%   out: Rotation vector, i.e. The magnitude of each element is the amount
%   of rotation in that direction (X,Y,Z)


    % If the input matrix is a rotation of 0, then it will be the I matrix:
    if norm(mat-eye(3)) < 1e-10
        out=[0 0 0]';
        return
    end
    
    % sr and srv used to create the rotation vector of the input rotation
    % matrix:
    sr = 1/2*(mat - transpose(mat));
    srv = [sr(3, 2) sr(1, 3) sr(2, 1)]';
    
    if norm(srv) < 1e-10
        % This is a 180 degree rotation, need to figure out which
        % direction:
        signs = [1 1 1; -1 1 1; 1 -1 1; -1 -1 1; 1 1 -1; -1 1 -1; 1 -1 -1; -1 -1 -1]';
        %   8 signs seem redundant, need to check this.
        %   xyz becomes the correct axis of rotation depending on mat:
        xyz = real([1/2*sqrt(1+mat(1,1)-mat(2,2)-mat(3,3)) 1/2*sqrt(1-mat(1,1)+mat(2,2)-mat(3,3)) 1/2*sqrt(1-mat(1,1)-mat(2,2)+mat(3,3))]');
        
        for ii=1:8
            % candout becomes the rotation vector:
            candout = pi*signs(:,ii).*xyz;
            % using rodrigues, find the rotation matrix that matches the
            % given rotation matrix. Thus outputting the correct rotation
            % vector:
            if norm(Rodrigues(candout)-mat) < 1e-6
                out = candout;
                return;
            end
        end
        % Not sure if this is just an error inclusion incase no 180 degree
        % rotation works. Ideally, out should be a vector.
        out = 0;
        return;
    end
    
    % If rotation is not 0 or 180, calculate the proper rotation about each
    % axis:
    %   1st: The norm of srv is the magnitude of the arcsin of rotation.
    %   2nd: normalizing srv gives the axis of rotation
    sinth = norm(srv);
    th = asin(sinth);
    xyz = Normalize(srv);
    
    if xyz(2) == 0 && xyz(3) == 0
        % Quick fix: if rotation is around x axis only, costh cannot be
        % used.
        

        if norm(mat-Rodrigues(th*xyz)) > 0.001
        %  Correct the magnitude of rotation if it is greater than pi in
        %  the X axis:
            th = pi-th;
        end
    else
        %   If the magnitude of rotation is greater than pi on anything
        %   other than the X axis use the following to check:
        costh = (-1+mat(1,1)+xyz(2)^2+xyz(3)^2)/(xyz(2)^2+xyz(3)^2);
        
        if costh < 0
            th = pi-th;
        end
    end
    
    out = th*xyz;
    
end

