function [tform valid_pair_num error] = getRigidTransform(new_pointcloud, ref_pointcloud, ref_normals)
    
    %==== Initialize parameters ====
    iter_num = 6;
    d_th = 0.05;
    m = size(new_pointcloud.Location, 1);
    n = size(new_pointcloud.Location, 2);
    tform = affine3d(eye(4));
    
    %==== Main iteration loop ====
    for iter = 1:iter_num
        %==== Set variables ====
        new_pts = new_pointcloud.Location;
        ref_pts = ref_pointcloud.Location;
        
        %==== For each reference point, find the closest new point within a local patch of size 3-by-3 ====        
        %==== (Notice: assoc_pts[] has the same size and format as new_pts[] and ref_pts[]) ====
        %==== (Notice: assoc_pts[i, j, :] = [0 0 0] iff no point in new_pts[] matches to ref_pts[i, j, :]) ====
        assoc_pts = findLocalClosest(new_pts, ref_pts, m, n, d_th);
        
        assoc_pts2dim = reshape(assoc_pts, m*n,3);
        ref_normals2dim  = reshape(ref_normals,m*n,3);
        ref_pts = reshape(ref_pts,m*n,3);
        new_pts2dim = reshape(new_pts,m*n,3);

        
        
        
        
        
        %==== Set the sizes of matrix A[] and vertor b[] of normal equation: A'*A*x = A'*b ====
        A = zeros(m*n, 6);
        b = zeros(m*n, 1);
        G = zeros(3,6,m*n);
        
        %==== declare the number of point pairs that are used in this iteration ==== 
       
        
        
        
        %==== TODO: Assign values to A[] and b[] ====
        %==== (Notice: the format of the desired 6-vector is: xi = [r_x r_y r_z t_x t_y t_z]') ====
        
        % Write your code here...
        
        count = 1;
        ref_normals2dim = ref_normals2dim';

        for i =1 : size(assoc_pts2dim,1)
            if ~(sum(abs(assoc_pts2dim(i,:))) == 0)
                
                temp = [toSkewSym(assoc_pts2dim(i,:)),eye(3)];
                A(i,:) = (temp'*(ref_normals2dim(:,i)));
                b(i,1) = ref_normals2dim(:,i)'*( -  assoc_pts2dim(i,:) + ref_pts(i,:))';
                count = count +1 ;

            end
            
        end
         valid_pair_num = count;
        %==== TODO: Solve for the 6-vector xi[] of rigid body transformation from A[] and b[] ====
        
        xi =  A\b;
      %  xi = -1*xi;
        
        % Write your code here...
        
        
        %==== Coerce xi[] back into SE(3) ====
        %==== (Notice: tmp_tform[] is defined in the format of right-multiplication) ====
        R = toSkewSym(xi(1:3)) + eye(3);
        [U,S,V] = svd(R);
        R = U*V';
        T = [R [0 ; 0 ; 0] ; [xi(4:6)' 1]];
        tmp_tform = affine3d(T);
        
        tform.T = (tmp_tform.T'*tform.T')';
        
        new_pointcloud = pctransform(new_pointcloud,tmp_tform);
        
        %==== TODO: Update the transformation and the pointcloud for the next iteration ====
        %==== (Hint: use affine3d() and pctransform() functions) ====
        %==== (Hint: be careful of the format of tform[] and the affine3d() function) ====
       
        % Write your code here...
        
    end
    
    %==== Find RMS error of point-plane registration ====
    error = sqrt(sum((A*xi - b).^2)/valid_pair_num);
end
        