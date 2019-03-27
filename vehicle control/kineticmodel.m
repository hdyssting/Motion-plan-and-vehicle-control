function [sys,x0,str,ts] = kineticmodel(t,x,u,flag)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitialSizes;
    case 2
        sys = mdlUpdates(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {1,4,9}
        sys = [];
    otherwise
        error(['unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitialSizes


function sys = mdlUpdates(t,x,u)


function sys = mdlOutputs(t,x,u)