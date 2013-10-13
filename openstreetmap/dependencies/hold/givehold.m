function [] = givehold(ax, held)
%GIVEHOLD   revert hold status
%
% input
%   ax = axes object handle
%   held = 1 to set hold(ax, 'on')
%   held = 0 to set hold(ax, 'off')
%
% See also TAKEHOLD, HOLD, ISHOLD.
%
% File:      givehold.m
% Author:    Ioannis Filippidis, jfilippidis@gmail.com
% Date:      2012.01.22 - 
% Language:  MATLAB R2011b
% Purpose:   revert hold status of axes object
% Copyright: Ioannis Filippidis, 2012-

if held == 1
    hold(ax, 'on')
elseif held == 0
    hold(ax, 'off')
else
    error('hold was neither on or off.')
end
