function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    new_pointcloud = pctransform(fusion_map.pointcloud,tform);
    new_pointcloudboolean = new_pointcloud.Location(:,3) > 0;
    new_pointcloudindices = find(new_pointcloudboolean);
        point_cloudxyz = fusion_map.pointcloud.Location(new_pointcloudindices,:);
        point_cloudxyzcamera = new_pointcloud.Location(new_pointcloudindices,:);
        pointnormals  = fusion_map.normals(new_pointcloudindices,:);
        pointcolors   = fusion_map.pointcloud.Color(new_pointcloudindices,:);
        pointcounts  = fusion_map.ccounts(new_pointcloudindices,:);
        pointtimes   = fusion_map.times(new_pointcloudindices,:);
        
        
    CameraInt = [fx 0 cx ; 0 fy cy ; 0 0 1  ];
    Transform = inv(tform.T');
    Transform = Transform(1:3,:);
%    point_cloudxyz = fusion_map.pointcloud.Location;
    point_cloudxyz(:,4) =  1;
    proj_map = CameraInt*Transform*point_cloudxyz';
    proj_map(2,:) = proj_map(2,:)./proj_map(3,:);
    proj_map(1,:) = proj_map(1,:)./proj_map(3,:);
    
    
    count =1 ;
    proj_map = round(proj_map);
    infrontofcamera = proj_map(3,:)>0;
    indexofcoordx = proj_map(1,:) < w & proj_map(1,:) > 0  ;
    indexofcoordy = proj_map(2,:) < h & proj_map(2,:) > 0 ;
    

    index = indexofcoordx.*indexofcoordy.*infrontofcamera;
    indices = find(index);
    row = proj_map(2,indices);
    col = proj_map(1,indices);
    
    proj_points = zeros(h*w,3);
    proj_normals = zeros(h*w,3);
    proj_colors = zeros(h*w,3);
    proj_ccounts = zeros(h*w,1);
    proj_times = zeros(h*w,1);
    sizematrix = zeros(h,w);
    index1 = sub2ind(size(sizematrix),row,col);
   
    proj_points(index1(:),:) = point_cloudxyz(indices(:),1:3);
    proj_points = reshape(proj_points,h,w,3);
    
    proj_normals(index1(:),:) = pointnormals(indices(:),1:3);
  %  proj_normals(:,4) = 0;
  %  proj_normas = tform.T'*proj_normals';
   % proj_normals = proj_normals(:,1:3);
    proj_normals = reshape(proj_normals,h,w,3);

    proj_colors(index1(:),:) = pointcolors(indices(:),:);
    proj_colors = reshape(proj_colors,h,w,3);
    
    proj_ccounts(index1(:),:) = pointcounts(indices(:));
    proj_ccounts = reshape(proj_ccounts,h,w);

    proj_times(index1(:),:)  = pointtimes(indices(:));
    proj_times = reshape(proj_times,h,w);
% 

  proj_flag =  index;

  
  
  

%     for j = 1 : size(row,2)
%         
%             proj_points(row(j),col(j),:) = point_cloudxyz(indices(j),1:3);
%             proj_normals(row(j),col(j),:) = fusion_map.normals(indices(j),:);
%             proj_colors(row(j),col(j),:) = fusion_map.pointcloud.Color(indices(j),:)';
%            proj_ccounts(row(j),col(j)) = fusion_map.ccounts(indices(j));
%            proj_times(row(j),col(j))  = fusion_map.times(indices(j));
%  
%             
%         
%     end
    
    
%     points1(row(:),col(:),1:4) = point_cloudxyz(indices(:),:);
%     
%     for  i = 1 : size(proj_map,2)
%         
%         pixelcoord = proj_map(:,i);
%         if (pixelcoord(1) < w && pixelcoord(2) <h)
%             
%             points(:,count) = pixelcoord;
%             colors(:,count) = fusion_map.pointcloud.Color(i,:)';
%             normals(:,count) = fusion_map.normals(i,:)';
%             
%             count = count +1 ;
%             
%             
%             
%             
%             
%         end
%      
%         
%     end
%     
    
    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====
    
    % Write your code here...
    
    
    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
  proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
end