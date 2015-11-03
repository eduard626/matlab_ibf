clear all
%Intersection Bisector Surface
%(((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)+(z0-x1)*(z0-z1))>0.05)
[vert_glass,face_glass]=read_off('sub_glass.off');
fprintf('Read obj1 with %i vertices\n',size(vert_glass,2))
[vert_tab,face_tab]=read_off('sub_table.off');
fprintf('Read obj2 with %i vertices\n',size(vert_tab,2))
% Get the tranlastion magnitude in y
translation_y=max(vert_tab(2,:))-min(vert_tab(2,:));
%  hold on
%  drawMesh(vert_tab',face_tab')
%  hold off
% % Centering objects
avg_x_tab=mean(vert_tab(1,:));
avg_x_glass=mean(vert_glass(1,:));
avg_z_tab=mean(vert_tab(3,:));
avg_z_glass=mean(vert_glass(3,:));
difference=avg_x_glass-avg_x_tab;
%translation_x_glass=avg_x_glass-sign(difference)*.5*difference;
%translation_x_tab=avg_x_tab-sign(difference)*.5*difference;
%new_vert_tab=vert_tab-[translation_x_tab*ones(1,size(vert_tab,2));translation_y*ones(1,size(vert_tab,2));zeros(1,size(vert_tab,2))];
%new_vert_glass=vert_glass-[translation_x_glass*ones(1,size(vert_glass,2));zeros(1,size(vert_glass,2));zeros(1,size(vert_glass,2))];
new_vert_tab=vert_tab-[avg_x_tab*ones(1,size(vert_tab,2));translation_y*ones(1,size(vert_tab,2));avg_z_tab*ones(1,size(vert_tab,2))];
new_vert_glass=vert_glass-[avg_x_glass*ones(1,size(vert_glass,2));zeros(1,size(vert_glass,2));avg_z_glass*ones(1,size(vert_glass,2))];
% Getting edges
%edges_tab=meshEdges(face_tab');
%edges_glass=meshEdges(face_glass');
% Compute and draw normals
%drawFaceNormals(new_vert_tab',edges_tab,face_tab')
%drawFaceNormals(new_vert_glass',edges_glass,face_glass')
% Get bounding box with built-in funtion
box3d=boundingBox3d([new_vert_glass';new_vert_tab']);
box_glass=boundingBox3d(new_vert_glass');
box_tab=boundingBox3d(new_vert_tab');
centre=[box3d(2)-box3d(1), box3d(4)-box3d(3),box3d(6)-box3d(5)];
sq_diff=(box3d(1)-box3d(2))^2+(box3d(3)-box3d(4))^2+(box3d(5)-box3d(6))^2;
box_diag=sqrt(sq_diff);
sphere_rad=1.5*(box_diag*.5);
fprintf('Sphere rad: %f \n',sphere_rad)
%drawBox3d(box3d)
%drawSphere(centre,sphere_rad)
hull_glass=convhull(new_vert_glass');
hull_tab=convhull(new_vert_tab');
all_points=[new_vert_glass new_vert_tab]';
fprintf('Computing Voronoi diagram....')
tic;
[V,C]=voronoin(all_points);
fprintf('done\n')
vert_id_array=[];
vert_id_array2=[];
fprintf('Cell to array 1....')
tic
for i=1:size(new_vert_glass,2)
    vert_id_array=[vert_id_array;C{i}'];
end
fprintf('done \nCell to array2....')
for i=1+size(new_vert_glass,2):size(new_vert_tab,2)
    vert_id_array2=[vert_id_array2;C{i}'];
end
fprintf('done\n')
toc
fprintf('Non repeated points....')
tic
vertices_one=V(sort(unique(vert_id_array)),:);
vertices_two=V(sort(unique(vert_id_array2)),:);
fprintf('done\n')
toc
fprintf('Bound points within sphere...')
tic
[points1,~]=bound(vertices_one,-sphere_rad,sphere_rad);
[points2,~]=bound(vertices_two,-sphere_rad,sphere_rad);
fprintf('done\n')
toc
fprintf('Pairwise distances....')
tic
distances=distancePoints(points1,points2);
fprintf('done\n')
toc
fprintf('Ridges in common....')
tic
id=find(min(distances)==0);
common=points2(id,:);
fprintf('done\n')
toc
scatter3(common(:,1),common(:,2),common(:,3),'.b');
hold on
drawMesh(new_vert_glass',face_glass')
hold on
drawMesh(new_vert_tab',face_tab')