function [held] = takehold(ax, mode)
%TAKEHOLD  hold axes and return previous hold status
%
% input
%   ax = axes object handle
%   mode = 'add' | 'local'
%
% output
%   held = 1 if ax already held
%          0 if ax not held
%
% See also GIVEHOLD, HOLD, ISHOLD.
%
% File:      takehold.m
% Author:    Ioannis Filippidis, jfilippidis@gmail.com
% Date:      2012.01.22 - 2012.04.24
% Language:  MATLAB R2011b
% Purpose:   hold(ax, 'on') and return current hold status
% Copyright: Ioannis Filippidis, 2012-

if nargin < 2
    mode = 'add';
end

held = ishold(ax);

if held == 1
    return
end

if strcmp(mode, 'local')
    cla(ax)
end

hold(ax, 'on')
