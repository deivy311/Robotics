function [Converted,lparamd,txy_] = optiomalSortD(vartoorder,allcombD)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[n,m]=size(vartoorder);
for i=1:n
    for j=1:m
        if(~isempty(symvar(vartoorder(i,j))))
            lparamd=[];
            txy_=[];
            [cxy, txy] = coeffs(simplify(collect(expand(vartoorder(i,j)))),allcombD , 'All');
            lparamd=simplify(cxy(:));
            txy_=txy(:);
            Converted(i,j)=sum(lparamd.*txy_);
        else
           Converted(i,j)= vartoorder(i,j);
        end
        
    end
end
Converted(i,j)=simplify(Converted(i,j));
end

