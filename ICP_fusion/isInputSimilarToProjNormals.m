function is_similar = isInputSimilarToProjNormals(input_normals, proj_normals, dot_th)

    %==== TODO: Output a boolean matrix which represents if each corresponding point normals are similar enough given dot_th ====    
    
    % Write your code here...
        
    i = 1
 %  a = input_normals./repmat((sqrt(input_normals(:,:,1).^2 + input_normals(:,:,2).^2 + input_normals(:,:,3).^2)),1,1,3);
 %  b = proj_normals./repmat((sqrt(proj_normals(:,:,1).^2 + proj_normals(:,:,2).^2 + proj_normals(:,:,3).^2)),1,1,3);
   
   is_similar = sum(input_normals.*proj_normals,3) > dot_th;
   
   
   

end
    