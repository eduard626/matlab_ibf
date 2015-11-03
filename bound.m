function [p,o]=bound(data,minLim,maxLim)
o=0;
bounded_data=zeros(size(data));
for i=1:size(data,1)
    for j=1:size(data,2)
        bounded_data(i,j)=data(i,j);
        if data(i,j)>maxLim
            bounded_data(i,j)=maxLim;
            o=1;
        end
        if data(i,j)<minLim
            bounded_data(i,j)=minLim;
            o=1;
        end
    end
end

p=bounded_data;