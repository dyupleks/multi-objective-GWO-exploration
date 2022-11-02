function z = DistanceCost(xy)
z = pdist([[xy(1),xy(2)];[xy(3),xy(4)]],'euclidean');
end
