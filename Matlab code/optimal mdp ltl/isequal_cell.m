function equal=isequal_cell(cell1,cell2)
equal=false;
for ii=1:length(cell1)
    found=false;
    for jj=1:length(cell2)
        if isequal(cell1{ii},cell2{jj})
            found=true;
            break
        end
    end
    if ~found
        return
    end
end

for ii=1:length(cell2)
    found=false;
    for jj=1:length(cell1)
        if isequal(cell2{ii},cell1{jj})
            found=true;
            break
        end
    end
    if ~found
        return
    end
end   
equal=true;
    