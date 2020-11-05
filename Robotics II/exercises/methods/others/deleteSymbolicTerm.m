function matrixReturn = deleteSymbolicTerm(matrix,term)
    for j =1: numel(matrix)
         if isequaln(matrix(j),term)
                matrix(j) = [];
                break
         end
    end
    matrixReturn = matrix;
end