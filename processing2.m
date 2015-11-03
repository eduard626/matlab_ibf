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
dmin=10;
d=0;
closest_idx=0;
debug=0;
for i=1:size(hull_glass,1)
    closest=zeros(1,3);
    idx=hull_glass(i,:);
    p1=[new_vert_glass(1,hull_glass(i,1)) new_vert_glass(2,hull_glass(i,2)) new_vert_glass(3,hull_glass(i,3))];
    scatter3(p1(1,1),p1(1,2),p1(1,3),'MarkerFace','y');
    hold on
    % Get the closest point
    for j=1:size(hull_tab,1)
        debug=debug+1;
        p2=[new_vert_tab(1,hull_tab(j,1)) new_vert_tab(2,hull_tab(j,2)) new_vert_tab(3,hull_tab(j,3))];
        d=distancePoints3d(p1,p2);
        if d<dmin
            dmin=d;
            closest=p2;
            closest_idx=j;
        end
    end
    scatter3(closest(1,2),closest(1,2),closest(1,3),'MarkerFace','g');
    hold on
    point=(p1+closest)/2;
    c=[closest;p1];
    plot3(c(:,1),c(:,2),c(:,3))
    hold on
    drawMesh(new_vert_glass',face_glass')
    hold on
    drawMesh(new_vert_tab',face_tab')
    hold on
    scatter3(point(1,1),point(1,2),point(1,3),'MarkerFace','b');
    hold on
    % Check for penetrations
    if ~inpolyhedron(hull_tab,new_vert_tab',closest)
        if ~inpolyhedron(hull_glass,new_vert_glass',closest)
            % Check that vector is not crossing object
            good_points(cnt,:)=[point i closest_idx];
            cnt=cnt+1;
        end
    end
    pause(0.5);
    hold off
end
good=good_points(1:cnt-1,:);
drawMesh(new_vert_glass',face_glass')
hold on
drawMesh(new_vert_tab',face_tab')
hold on
scatter3(good(:,1),good(:,2),good(:,3),'b.','MarkerFaceColor','b')
    