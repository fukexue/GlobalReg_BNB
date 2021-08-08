function ptCloudOut = pc_downsample(pt,gridStep)
   
ptCloudIn=pointCloud(pt');


ptCloudOut = pcdownsample(ptCloudIn,'gridAverage',gridStep);

ptCloudOut=ptCloudOut.Location;

end







    
    