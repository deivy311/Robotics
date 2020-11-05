%Autor DAvid Esteban Imbajoa Ruiz
function [JacobianSinFac,JacobianSinFac_s] = JacobianSingularities(J,n)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    d_J=det(J);
    sincos_ = getShortNotation_SinCos(n);
    JacobianSinFac=simplify(factor(d_J));
    JacobianSinFac_s=toShortNotation(JacobianSinFac,sincos_);
end

