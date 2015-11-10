function plotSimulationResults(points, colors)
  hold off;
  for index = 2:length(points)
    points(index,1);
    plot(points(index-1:index,1), points(index-1:index,2), 'Color', colors(index,:));
  end
  axis equal;
end
