function struct_str = printStruct(S)
%STRUCT_STR - returns a string matrix of a structure with symbolic elements
% Syntax:  struct_str =  printStruct(S)
%
% Inputs:
%    S = Structure of symbolic elements
%
% Outputs:
%    S_str = string version of the structure
%
% Author: Andres Arciniegas - 
% email: arciandres@gmail.com -
% Last revision: 08-Jul-2019

for i = 1:length(S)
    names = fieldnames(S(i));
    for j = 1:length(names)
        struct_str(i,j) = string(char(getfield(S(i),names{j})));    
    end
end
