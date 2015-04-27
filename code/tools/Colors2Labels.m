function labels = Colors2Labels(colors,cm)

    if max(cm(:))<2
        cm = round(255*cm);
    end
    labels = knnsearch(cm,double(colors))-1;
  
end