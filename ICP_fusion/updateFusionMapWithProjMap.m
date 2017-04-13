function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====
    
    % Write your code here...
    updated_map.points2dim = reshape(updated_map.points,h*w,3);
    updated_map.colors2dim   = reshape(updated_map.colors,h*w,3);
    indices = (find(~proj_flag(1,:)));
    updated_map.times = reshape(updated_map.times,h*w,1);

   map_points =  fusion_map.pointcloud.Location(indices(:),:);
   map_points = cat(1,map_points,updated_map.points2dim);
   
   
   map_colors =  fusion_map.pointcloud.Color(indices(:),:);
   map_colors = cat(1,map_colors,updated_map.colors2dim);
   
   map_normals = fusion_map.normals(indices(:),:);
   map_normals= cat(1,map_normals,reshape(updated_map.normals,h*w,3));
   
  
   map_ccounts =fusion_map.ccounts(indices(:),:);
   map_ccounts= cat(1,map_ccounts,updated_map.ccounts);
   
   
   map_times = fusion_map.times(indices(:),:);
   map_times= cat(1,map_times,updated_map.times);

   
   
    
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   