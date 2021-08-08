function ptCloudOut = pc_downsample_2d(pt,gridStep)
  
pt=[pt;ones(1,size(pt,2))];
ptCloudIn=pointCloud(pt');


ptCloudOut = pcdownsample(ptCloudIn,'gridAverage',gridStep);

ptCloudOut=ptCloudOut.Location;
ptCloudOut=ptCloudOut(:,[1,2]);

end







    
    