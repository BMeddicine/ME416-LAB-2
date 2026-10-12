function pointUV_mm = applyTagOffset(centerUV_mm, yaw_deg, offsetTag_mm)
%APPLYTAGOFFSET Point given in a tag's own frame -> world [U, V].
%   pointUV_mm = applyTagOffset(centerUV_mm, yaw_deg, offsetTag_mm) rotates
%   the offset [x, y] (tag frame: x along corner 0 -> 1, y 90 deg CCW) by
%   the tag's yaw and adds it to the tag centre. Same as hw4B3.m.

    rot = [cosd(yaw_deg) -sind(yaw_deg); sind(yaw_deg) cosd(yaw_deg)];
    pointUV_mm = centerUV_mm(:).' + (rot * offsetTag_mm(:)).';
end
