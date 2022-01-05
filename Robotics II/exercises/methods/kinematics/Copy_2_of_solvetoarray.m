function [qdvarray_,qdvnames_,vartosolve2] = solvetoarray(qdv_,vartosolve)



qdvarray_=[];
qdvnames_=[];
pinam=fieldnames(qdv_);
for pind=1:size(pinam,1)
    qdvarray_=[qdvarray_;simplify(qdv_.(pinam{pind}))'];
    qdvnames_=[qdvnames_;string(pinam{pind})];
    
end

qdvarray_=simplify(qdvarray_);

if exist('vartosolve')
    % third parameter does not exist, so default it to something
    
    [tf,loc]=ismember(string(vartosolve),qdvnames_);
    for iv=1:size(tf,1)
            tempvarfg=sym(string(vartosolve(iv,1)),[1,size(qdvarray_,2),],'real');
        if(tf(iv)>0)
            vartosolve2(iv,:)=qdvarray_(loc(iv),:);
        else
            vartosolve2(iv,:)=tempvarfg;
        end
    end
end
end

