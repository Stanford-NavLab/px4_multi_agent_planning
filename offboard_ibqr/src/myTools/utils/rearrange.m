function mat = rearrange(arr)

    splitIdx = find(isnan(arr(:,1)));
    
    mat = cell(1,length(splitIdx));
    
    for i=1:length(splitIdx)
        
        if(i == 1)
            mat{i} = arr(1:splitIdx(1)-1, :);
        else
            mat{i} = arr(splitIdx(i-1)+1:splitIdx(i)-1, :);
        end
        
    end
end