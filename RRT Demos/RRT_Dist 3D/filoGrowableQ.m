classdef filoGrowableQ < handle

% The MIT License (MIT)
%
% Copyright June, 2019 Michael Otte, Universtiy of Maryland
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.



    % Copyright 2019 Michael Otte, University of Maryland
    
    % impliments a first-in-last-out queue (i.e., a stack)
    % note that update() does nothing, and is just here for compatibility.
        
    % this is a very simple version, but it will automatically double in 
    % storage size when it gets too small 
    
    properties
        Q          % an array of handles to structs of some valid node type (see notes above)
        last       % index of last
        totalSize  % current total size (including unused slots)
    end
    methods
        function obj = filoGrowableQ(numElements)
            % numElements is the default number of elements we think will
            % be inserted, but it will grow later if necessary
            
            obj.Q = cell(numElements,1);
            obj.last = 0;
            obj.totalSize = numElements;
        end
        function push(obj,el,varargin)   
            % el is some element we want to insert
            % additional arguments (varagin) enables compatibility with 
            % other types of queues (and are just ignored here)
            
            if obj.last == obj.totalSize
                % first need to grow the size by 2X
                obj.Q(obj.totalSize+1:2*obj.totalSize) = cell(obj.totalSize,1);
                obj.totalSize = 2*obj.totalSize;
            end
            
            obj.last = obj.last + 1;
            obj.Q{obj.last} = el;
        end
        function el = top(obj)
           if obj.last > 0            
              el = obj.Q{obj.last};
           else
              el = nan; 
           end 
        end
        function el = pop(obj)
           if obj.last > 0            
              el = obj.Q{obj.last};
              obj.last = obj.last - 1;
           else
              el = nan; 
           end  
        end
        function update(obj, varargin)   
           % DO nothing
           % this is hear to allow drop in compatibility with other forms
           % of queues
            
        end
        function ret = isEmpty(obj)
            % returns true if the heap is empty
            
            if obj.last > 0
                ret = false;
            else 
                ret = true;
            end     
        end
    end
end

