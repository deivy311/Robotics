% This method removed terms that might be contained in others.
% For instance depen = [2*a, a], a gets removed, and returns depenSubs=[a]
% Should be applied only if the coefficient is an scalar
% OnlyScalars: If the pair is [2*r*m, m], it lets it pass, because there is
% other symbolic value

function depenSubs = removeTermsSelfContained2(depen)
depenSubs = depen;
for i_coeffs=1:size(depenSubs,2)
    depenSubs(i_coeffs)=depenSubs(i_coeffs)/coeffs(depenSubs(i_coeffs));
end
depenSubs=unique(depenSubs);
% done = 1;
% while done
%     if length(depenSubs) > 1
%        depencombs = nchoosek(depenSubs,2);
%        [numpairs , two] = size(depencombs) ;
%     else
%         disp('Only one term in the list of terms')
%        break 
%     end
%     for i = 1:numpairs
%         pair = depencombs(i,:);
%         % If one term is contained in the other, it gets removed, since it
%         % will get replaced eventually
%         if (has(pair(1),pair(2)))
%             if isnan(str2double(char(pair(1)/pair(2)))) % 'NaN' if there is another symbolic variable, otherwise it should return double
%                 if i == numpairs
%                     done = 0;
%                 end
%                 continue % it does not delete the expression
%             end
%             depenSubs = deleteSymbolicTerm(depenSubs, pair(1));
%             break
%         end
%         if (has(pair(2),pair(1)))
%             if isnan(str2double(char(pair(1)/pair(2)))) % 'NaN' if there is another symbolic variable, otherwise it should return double
%                 if i == numpairs
%                     done = 0;
%                 end
%                 continue % it does not delete the expression
%             end
%             depenSubs = deleteSymbolicTerm(depenSubs, pair(2));
%             break
%         end
%         if i == numpairs
%             done = 0;
%         end
%         
%     end 
end


