function Hh = homographyAtHeight(pose, height_mm)
%HOMOGRAPHYATHEIGHT H_h: the plane W = h to undistorted pixel homography.
%   Hh = homographyAtHeight(pose, height_mm) with pose from
%   estimateTablePose. A point [U, V, h] projects to Hh * [U; V; 1], so
%   Hh = K * [r1 r2 (h*r3 + t)]. height_mm = 0 gives the table H.
%   Map pixels back to the plane with applyHomography(inv(Hh), px).

    R = pose.R;
    Hh = pose.K * [R(:,1) R(:,2) height_mm * R(:,3) + pose.t];
    Hh = Hh / Hh(3,3);
end
