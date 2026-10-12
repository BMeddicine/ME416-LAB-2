function out = applyHomography(H, in)
%APPLYHOMOGRAPHY Map Nx2 points through a 3x3 homography (with the divide).
%   out = applyHomography(H, in): out(k,:) = dehomogenized H * [in(k,:) 1]'.
%   Use H to go plane -> pixel, inv(H) to go pixel -> plane.

    outH = H * [in.'; ones(1, size(in, 1))];
    out = (outH(1:2,:) ./ outH(3,:)).';
end
