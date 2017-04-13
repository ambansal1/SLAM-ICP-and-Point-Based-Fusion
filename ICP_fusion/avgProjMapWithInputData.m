function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;
     proj_points2dim = reshape(proj_map.points,h*w,3);
     proj_normals2dim = reshape(proj_map.normals,h*w,3);
    proj_counts2dim = reshape(proj_ccounts,h,w);
    proj_colors2dim = reshape(proj_colors,h*w,3);
    input_colors2dim = reshape(input_colors,h*w,3);
   
    
    
    numeratorpos = repmat(proj_counts2dim,1,1,3).*proj_points + repmat(alpha,1,1,3).*input_points ;
    denominatorpos = repmat(proj_counts2dim,1,1,3) + repmat(alpha,1,1,3);
    
    kinectupdatepos = numeratorpos./denominatorpos;
     kinectupdatepos= reshape(kinectupdatepos,h*w,3);
    indices = find(is_use);
    proj_points2dim(indices,:) =  kinectupdatepos(indices,:);
    updated_points = reshape(proj_points2dim,h,w,3);
    
    numeratornorm = repmat(proj_counts2dim,1,1,3).*proj_normals +repmat(alpha,1,1,3).*input_normals ;
    denominatornorm = repmat(proj_counts2dim,1,1,3) + repmat(alpha,1,1,3);
    
    kinectupdatenorm = numeratornorm./denominatornorm;
     kinectupdatenorm= reshape(kinectupdatenorm,h*w,3);
    proj_normals2dim(indices,:) =  kinectupdatenorm(indices,:);
    updated_normals = reshape(proj_normals2dim,h,w,3);
    
    
    updated_colors = proj_colors2dim;
    updated_colors(indices,:)= input_colors2dim(indices,:);
    updated_colors = reshape(updated_colors,h,w,3);
    
    
    
    countupdate = proj_counts2dim + alpha;
    proj_counts2dim(indices(:)) = countupdate(indices(:));
    updated_ccounts = reshape(proj_counts2dim,h*w,1);
    
    
    
    timeupdate   = proj_times + t;
    updated_times = proj_times;
    updated_times(indices) = timeupdate(indices);
    
    
    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====

    % Write your code here...
    
    
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
    
end