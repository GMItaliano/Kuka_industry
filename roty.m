%ROTY Rotation about Y axis
%
% R = ROTY(THETA) is an SO(3) rotation matrix (3x3) representing a rotation of THETA 
% radians about the y-axis.
%
% R = ROTY(THETA, 'deg') as above but THETA is in degrees.
%
% See also ROTX, ROTZ, ANGVEC2R, ROT2.



% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function R = roty(t)
    ct = cos(t);
    st = sin(t);
    R = [
        ct  0   st
        0   1   0
       -st  0   ct
       ];
