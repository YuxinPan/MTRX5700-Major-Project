
%% View scan map
figure();
for ii = 1:length(scans)
    plot(scans{1,ii}.Cartesian(:,1), scans{1,ii}.Cartesian(:,2), 'b.');
    grid on;
    axis equal;
    axis([-5 5 0 8]);
    pause(0.5);
end