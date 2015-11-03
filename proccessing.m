clear all
%(((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)+(z0-x1)*(z0-z1))>0.05)
[vert_tab,face_tab]=read_off('sub_table.off');
%[vert_glass,face_glass]=read_off('sub_glass.off');
[vert_glass,face_glass]=read_off('sub_bottle.off');
% Get the tranlastion magnitude in y
translation_y=max(vert_tab(2,:))-min(vert_tab(2,:));
%  drawMesh(vert_glass',face_glass')
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
edges_tab=meshEdges(face_tab');
edges_glass=meshEdges(face_glass');
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
sphere_rad=1.5*box_diag;
%drawBox3d(box3d)
%drawSphere(centre,sphere_rad)
hull_glass=convhull(new_vert_glass');
hull_tab=convhull(new_vert_tab');
sample_points_ratio=0.6;
n_points_glass=zeros(round(sample_points_ratio*size(hull_glass,1)),3);
n_points_tab=zeros(round(sample_points_ratio*size(hull_tab,1)),3);
good_points=10*ones(size(n_points_glass,1)*size(n_points_tab,1),5);
cnt=1;
glass_idx=randperm(size(hull_glass,1));
tab_idx=randperm(size(hull_tab,1));
for i=1:size(n_points_glass,1)
    %idx=hull_glass(randi([1,size(hull_glass,1)],1),:);
    idx=glass_idx(i);
    n_points_glass(i,:)=new_vert_glass(:,idx)';
    %n_points_glass(i,:)=[new_vert_glass(1,idx(1)),new_vert_glass(2,idx(2)),new_vert_glass(3,idx(3))];
    for j=1:size(n_points_tab,1)
        %idx2=hull_tab(randi([1,size(hull_tab,1)],1),:);
        idx2=tab_idx(j);
        n_points_tab(j,:)=new_vert_tab(:,idx2)';
        %n_points_tab(j,:)=[new_vert_tab(1,idx2(1)),new_vert_tab(2,idx2(2)),new_vert_tab(3,idx2(3))];
        point=(n_points_glass(i,:)+n_points_tab(j,:))/2;
        if ~inpolyhedron(hull_tab,new_vert_tab',point)
            if ~inpolyhedron(hull_glass,new_vert_glass',point)
                good_points(cnt,:)=[point idx idx2];
%                 scatter3(good_points(cnt,1),good_points(cnt,2),good_points(cnt,3),'b')
%                 hold on
%                 scatter3(n_points_glass(i,1),n_points_glass(i,2),n_points_glass(i,3),'g','MarkerFaceColor','g');
%                 hold on
%                 scatter3(n_points_tab(j,1),n_points_tab(j,2),n_points_tab(j,3),'g','MarkerFaceColor','g');
%                 input('')
                cnt=cnt+1;
            end
        end
    end
end
good=good_points(1:cnt-1,:);
drawMesh(new_vert_glass',face_glass')
hold on
drawMesh(new_vert_tab',face_tab')
hold on
scatter3(new_vert_glass(1,good(:,4)),new_vert_glass(2,good(:,4)),new_vert_glass(3,good(:,4)),'MarkerFaceColor','c')
hold on
scatter3(new_vert_tab(1,good(:,5)),new_vert_tab(2,good(:,5)),new_vert_tab(3,good(:,5)),'MarkerFaceColor','g')
hold on
scatter3(good(:,1),good(:,2),good(:,3),'b.','MarkerFaceColor','b')
%hull_vo=convhull(good(:,1:3));
%trisurf(hull_vo,good(:,1),good(:,2),good(:,3),'FaceColor','cyan','FaceAlpha',0.3)