function plot_cell(i,points,defaultFaceColor)
trisurf(h,points(:,1),points(:,2),points(:,3), 'FaceColor', defaultFaceColor, 'FaceAlpha',0.3)
hold on
scatter3(all_points(i,1),all_points(i,2),all_points(i,3),'MarkerFaceColor','b')
hold on
scatter3(points(:,1),points(:,2),points(:,3),'MarkerFaceColor','g')