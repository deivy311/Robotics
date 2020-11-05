%Get DH parameter table for Robotics toolbox

function dhtable = getDHforTB(dh,firstParam)

if ~exist('firstParam','var')
 % If it's not provided, default is defined
  firstParam = 'a';
end
 
%Example of output table
 
% noname:: 6 axis, RRRRRR, stdDH, slowRNE                          
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|       89.2|          0|     1.5708|          0|
% |  2|         q2|          0|        425|          0|          0|
% |  3|         q3|          0|        392|          0|          0|
% |  4|         q4|      109.3|          0|     1.5708|          0|
% |  5|         q5|      94.75|          0|    -1.5708|          0|
% |  6|         q6|       82.5|          0|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+
%  
n = size(dh,1)
if strcmp(firstParam,'a')
    dhtable = eval([zeros(n,1) dh(:,3) dh(:,1) dh(:,3)]);
end
if strcmp(firstParam,'alpha')
    dhtable = eval([zeros(n,1) dh(:,3) dh(:,2) dh(:,1)]);
end
