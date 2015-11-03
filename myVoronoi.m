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
%Useful to detect points sampled from different objects
obj_index=[zeros(size(new_vert_glass,2),1);ones(size(new_vert_tab,2),1)];
% For every cell in the voronoi diagram the number of riges(faces) tells
% the number of neighbouring points. Look for neighbours from different
% objects
%faces={};
random_glass=randperm(size(new_vert_glass,2));
random_tab=randperm(size(new_vert_tab,2));
glass_sample_size=round(0.4*size(new_vert_glass,2));
tab_sample_size=round(0.4*size(new_vert_tab,2));
ob1_idx=size(new_vert_glass,2);
ob2_idx=size(new_vert_tab,2);
cnt=1;
cnt_max=size(V,1);
shared_v=zeros(cnt_max,3);
fprintf('Computing ridges between different objects ...')
for i=1:glass_sample_size
    vertex_ob1=C{random_glass(i)};
    points_ob1=V(vertex_ob1,:);
    [points1,~]=bound(points_ob1,-sphere_rad,sphere_rad);
    for j=ob1_idx+1:tab_sample_size+ob1_idx
        vertex_ob2=C{random_tab(j-ob1_idx)+ob1_idx};
        points_ob2=V(vertex_ob2,:);
        [points2,~]=bound(points_ob2,-sphere_rad,sphere_rad);
        distances=distancePoints(points1,points2);
        id=find(min(distances)==0);
        n=length(id);
        if n>0
            if (cnt+n-1)>cnt_max
                cnt_max=cnt_max+size(V,1);
                shared_v(cnt:cnt_max,:)=0;
            end
            shared_v(cnt:cnt+n-1,:)=points2(id,:);
            cnt=cnt+n;
            %shared_v=[shared_v;points2(id,:)];
            %faces{cnt}=points2(id,:);
        end
    end
end
[~, I, ~] = unique(shared_v,'first','rows');
aux=sort(I);
good_surface=shared_v(aux,:);
fprintf('done \n')
toc
fprintf('Found %i points\n',size(good_surface,1))
%  for f=1:size(faces,2)
%      ridge=faces{f};
%      fill3(ridge(:,1),ridge(:,2),ridge(:,3),'c')
%       pause(0.5)
%      hold on
%  end
scatter3(good_surface(:,1),good_surface(:,2),good_surface(:,3),'.b')
hold on
drawMesh(new_vert_glass',face_glass')
hold on
drawMesh(new_vert_tab',face_tab')