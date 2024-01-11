function outStructArray = sort_array_of_struct( structArray, fieldName )

if ( ~isempty(structArray) &&  ~isempty (structArray))
    vSortValues = arrayfun (@(x) x.(fieldName), structArray);
    [~,I] = sort(vSortValues) ;
    outStructArray = structArray(I) ;
else
    disp ('Array of struct is empty');
end

end
